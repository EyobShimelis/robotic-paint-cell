#!/usr/bin/env python3
"""Splits the detailed car display mesh into paint panels (paint_<zone>.stl) and the unpainted rest
(car_body.stl), so each panel can be shown and measured on its own. A triangle belongs to a panel if it lies
on that zone's simplified mesh (meshes/zones/) or is visible from that side of the car.
Re-runnable: it starts from car_body.stl + the existing paint_*.stl.

    python3 scripts/split_paint_zones.py
"""
import glob
import os
import struct

import numpy as np
from scipy.spatial import cKDTree

TOL = 0.02            # a display triangle belongs to a zone if its centre is within 2 cm of the zone mesh
REC = np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])


def load(path):
    b = open(path, 'rb').read()
    n = struct.unpack('<I', b[80:84])[0]
    return np.frombuffer(b[84:84 + n * 50], dtype=REC)['v'].reshape(-1, 3, 3).astype(float)


def save_double_sided(path, tris, name):
    t = np.concatenate([tris, tris[:, [0, 2, 1]]]).astype('<f4')
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(t), dtype=REC)
    rec['n'], rec['v'] = n, t.reshape(-1, 9)
    with open(path, 'wb') as fh:
        fh.write(name.encode().ljust(80, b' ') + struct.pack('<I', len(rec)) + rec.tobytes())


def unique_triangles(t):
    """Drop the reversed copies of a double-sided mesh."""
    _, vid = np.unique(np.round(t.reshape(-1, 3), 5), axis=0, return_inverse=True)
    _, idx = np.unique(np.sort(vid.reshape(-1, 3), axis=1), axis=0, return_index=True)
    return t[np.sort(idx)]


def orient_outward(t):
    """Flip triangles so their normal points away from the car's centre axis."""
    c = t.mean(1)
    ref = np.c_[np.clip(c[:, 0], -1.6, 1.6), np.zeros(len(c)), np.full(len(c), 0.75)]
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    flip = np.einsum('ij,ij->i', n, c - ref) < 0
    t = t.copy()
    t[flip] = t[flip][:, [0, 2, 1]]
    return t


def sample_surface(t, spacing=0.01):
    """Points spread over a mesh, about one per spacing^2 of area."""
    area = np.linalg.norm(np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]), axis=1) / 2
    n = np.maximum(1, (area / spacing ** 2).astype(int))
    rng = np.random.default_rng(0)
    idx = np.repeat(np.arange(len(t)), n)
    w = rng.dirichlet([1, 1, 1], size=len(idx))
    return np.einsum('ij,ijk->ik', w, t[idx])


def visible_from(centre, cloud, axis, sign, pixel=0.015, tolerance=0.03):
    """Triangle centres visible along one axis (sign +1: seen from +axis), via a depth image of the car."""
    u, v = [a for a in range(3) if a != axis]
    lo = cloud[:, [u, v]].min(0)
    shape = np.ceil((cloud[:, [u, v]].max(0) - lo) / pixel).astype(int) + 1
    ij = ((cloud[:, [u, v]] - lo) / pixel).astype(int)
    depth = np.full(shape, -np.inf)
    np.maximum.at(depth, (ij[:, 0], ij[:, 1]), sign * cloud[:, axis])
    cij = np.clip(((centre[:, [u, v]] - lo) / pixel).astype(int), 0, shape - 1)
    return sign * centre[:, axis] >= depth[cij[:, 0], cij[:, 1]] - tolerance


# panel: (view axis, sign, min |normal component|, region test) for the parts outside the zone meshes
EXTEND = {
    'roof':       (2, +1, 0.50, lambda c: (c[:, 2] > 1.25) & (c[:, 0] < 0.80)),
    'hood':       (2, +1, 0.40, lambda c: (c[:, 2] > 0.60) & (c[:, 2] <= 1.25) & (c[:, 0] > 0.90)),
    'left_side':  (1, +1, 0.35, lambda c: c[:, 1] > 0.30),
    'right_side': (1, -1, 0.35, lambda c: c[:, 1] < -0.30),
    'rear':       (0, -1, 0.35, lambda c: c[:, 0] < -1.50),
    'front':      (0, +1, 0.35, lambda c: c[:, 0] > 1.60),
}


if __name__ == '__main__':
    pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    vis = os.path.join(pkg, 'meshes', 'visual')
    parts = [load(os.path.join(vis, 'car_body.stl'))] + [load(p) for p in sorted(glob.glob(os.path.join(vis, 'paint_*.stl')))]
    body = orient_outward(unique_triangles(np.concatenate(parts)))
    centre = body.mean(1)
    normal = np.cross(body[:, 1] - body[:, 0], body[:, 2] - body[:, 0])
    area = np.linalg.norm(normal, axis=1) / 2
    normal /= 2 * area[:, None] + 1e-12

    # 1. triangles lying on a planning zone
    names = list(EXTEND)
    dist = np.full((len(names), len(body)), np.inf)
    for i, z in enumerate(names):
        path = os.path.join(pkg, 'meshes', 'zones', z + '.stl')
        if os.path.exists(path):
            dist[i], _ = cKDTree(sample_surface(load(path))).query(centre)
    label = np.where(dist.min(0) < TOL, dist.argmin(0), -1)
    label[(label == names.index('left_side')) & (centre[:, 1] < 0)] = -1
    label[(label == names.index('right_side')) & (centre[:, 1] > 0)] = -1
    on_zone = label.copy()

    # 2. the rest of the visible body, by direction
    others = [unique_triangles(load(os.path.join(vis, f))) for f in ('car_glass.stl', 'car_trim.stl')]
    cloud = sample_surface(np.concatenate([body] + others), spacing=0.008)
    score = np.zeros(len(body))
    for i, z in enumerate(names):
        axis, sign, min_n, region = EXTEND[z]
        facing = np.abs(normal[:, axis])
        ok = (label == -1) | ((on_zone == -1) & (facing > score))
        ok &= visible_from(centre, cloud, axis, sign) & (facing > min_n) & region(centre)
        label[ok], score[ok] = i, facing[ok]

    save_double_sided(os.path.join(vis, 'car_body.stl'), body[label == -1], 'car body, unpainted parts')
    print(f"{'car_body (unpainted)':22s} {int((label == -1).sum()):6d} tris  {area[label == -1].sum():6.2f} m2")
    for i, z in enumerate(names):
        m = label == i
        save_double_sided(os.path.join(vis, f'paint_{z}.stl'), body[m], f'paint panel {z}')
        print(f'{"paint_" + z:22s} {int(m.sum()):6d} tris  {area[m].sum():6.2f} m2   (on the planning zone: {area[on_zone == i].sum():.2f} m2)')
