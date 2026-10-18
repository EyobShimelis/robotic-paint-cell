"""Mesh and path geometry for the paint planner (numpy only, car frame unless stated)."""
import struct

import numpy as np

STL_REC = np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')])
AXIS = {'x': 0, 'y': 1, 'z': 2}


def load_stl(path):
    """Binary STL -> (n, 3, 3) array of triangle vertices."""
    with open(path, 'rb') as fh:
        b = fh.read()
    n = struct.unpack('<I', b[80:84])[0]
    return np.frombuffer(b[84:84 + n * 50], dtype=STL_REC)['v'].reshape(-1, 3, 3).astype(float)


def save_stl(path, tris):
    t = np.asarray(tris, dtype='<f4')
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(t), dtype=STL_REC)
    rec['n'], rec['v'] = n, t.reshape(-1, 9)
    with open(path, 'wb') as fh:
        fh.write(b'paint'.ljust(80, b' ') + struct.pack('<I', len(rec)) + rec.tobytes())


def unit(v):
    v = np.asarray(v, dtype=float)
    return v / (np.linalg.norm(v, axis=-1, keepdims=True) + 1e-12)


def triangle_normals(tris):
    return unit(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]))


def triangle_areas(tris):
    return np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1) / 2


def orient_outward(tris):
    """Flip triangles to face away from the car's centre axis (about half of the CAD surfaces face inward)."""
    c = tris.mean(1)
    ref = np.c_[np.clip(c[:, 0], -1.6, 1.6), np.zeros(len(c)), np.full(len(c), 0.75)]
    flip = np.einsum('ij,ij->i', np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), c - ref) < 0
    tris = tris.copy()
    tris[flip] = tris[flip][:, [0, 2, 1]]
    return tris


def subdivide(tris, max_edge):
    """Split triangles at the midpoint of their longest edge until no edge is longer than max_edge."""
    tris = np.asarray(tris, dtype=float)
    for _ in range(20):
        edges = np.linalg.norm(tris[:, [1, 2, 0]] - tris, axis=2)
        k = edges.argmax(1)
        big = edges.max(1) > max_edge
        if not big.any():
            break
        t, k = tris[big], k[big]
        r = np.arange(len(t))
        a, b, c = t[r, k], t[r, (k + 1) % 3], t[r, (k + 2) % 3]
        m = (a + b) / 2
        tris = np.concatenate([tris[~big], np.stack([a, m, c], 1), np.stack([m, b, c], 1)])
    return tris


def ray_cast(origins, direction, tris):
    """First hit of parallel rays on a mesh (Moller-Trumbore). Returns (hit distance or nan, triangle index)."""
    d = np.asarray(direction, dtype=float)
    v0, e1, e2 = tris[:, 0], tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    p = np.cross(d, e2)
    det = np.einsum('ij,ij->i', e1, p)
    ok = np.abs(det) > 1e-12
    inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
    dist = np.full(len(origins), np.nan)
    index = np.full(len(origins), -1)
    for i, o in enumerate(origins):
        s = o - v0
        u = np.einsum('ij,ij->i', s, p) * inv
        q = np.cross(s, e1)
        v = (q @ d) * inv
        t = np.einsum('ij,ij->i', q, e2) * inv
        hit = ok & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9) & (t > 1e-6)
        if hit.any():
            k = np.where(hit)[0][np.argmin(t[hit])]
            dist[i], index[i] = t[k], k
    return dist, index


def rotation_from_tool_axes(z_axis, x_hint, roll=0.0):
    """Rotation matrix of the tool frame: Z = spray direction, X = x_hint made perpendicular to Z, then rolled about Z."""
    z = unit(z_axis)
    x = unit(x_hint - np.dot(x_hint, z) * z)
    y = np.cross(z, x)
    if roll:
        c, s = np.cos(roll), np.sin(roll)
        x, y = c * x + s * y, -s * x + c * y
    return np.column_stack([x, y, z])


def quaternion_from_matrix(m):
    """Rotation matrix -> quaternion (x, y, z, w)."""
    t = np.trace(m)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
        q = [0.0, 0.0, 0.0, (m[k, j] - m[j, k]) / s]
        q[i], q[j], q[k] = 0.25 * s, (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s
    return np.array(q)


def limit_tilt(direction, nominal, max_tilt):
    """Clamp the angle between a spray direction and the nominal approach direction."""
    d, n = unit(direction), unit(nominal)
    cosang = np.clip(d @ n, -1, 1)
    ang = np.arccos(cosang)
    if ang <= max_tilt:
        return d
    side = unit(d - cosang * n)
    return np.cos(max_tilt) * n + np.sin(max_tilt) * side


def raster_path(tris, approach, stripe_axis, step_axis, spacing=0.15, sample=0.05, max_tilt=np.radians(20),
                step_range=None, edge_margin=0.02, smooth=4, n_paint=None):
    """Zig-zag path over a zone: rays cast along `approach`, stripes along `stripe_axis`.
    Only the first n_paint triangles are sprayed (the rest, e.g. glass, bridge openings).
    Returns waypoints {p: surface point (= TCP), d: spray direction, spray, stripe}."""
    approach = unit(approach)
    sa, st = AXIS[stripe_axis], AXIS[step_axis]
    n_paint = len(tris) if n_paint is None else n_paint
    verts = tris[:n_paint].reshape(-1, 3)
    lo, hi = verts.min(0), verts.max(0)
    s_lo, s_hi = lo[st] + edge_margin, hi[st] - edge_margin
    if step_range is not None:
        s_lo, s_hi = max(s_lo, step_range[0]), min(s_hi, step_range[1])
    n_stripes = max(1, int(np.ceil((s_hi - s_lo) / spacing)))
    steps = s_lo + (np.arange(n_stripes) + 0.5) * (s_hi - s_lo) / n_stripes
    along = np.arange(lo[sa] + edge_margin, hi[sa] - edge_margin + 1e-9, sample)
    normals = triangle_normals(tris)
    far = verts.mean(0) - approach * 5.0            # start plane of the rays, well outside the car
    path, n_done = [], 0
    for k, s in enumerate(steps):
        origins = np.tile(far, (len(along), 1))
        origins[:, sa], origins[:, st] = along, s
        dist, idx = ray_cast(origins, approach, tris)
        hit = ~np.isnan(dist)
        hit &= np.einsum('ij,j->i', normals[np.where(idx >= 0, idx, 0)], -approach) > 0.5   # skip edges seen side-on
        if hit.sum() < 4:
            continue
        first, last = np.where(hit)[0][[0, -1]]
        pts = origins + np.nan_to_num(dist)[:, None] * approach
        nrm = np.where(hit[:, None], normals[np.where(idx >= 0, idx, 0)], -approach)
        ii = np.arange(first, last + 1)
        good = ii[hit[ii]]
        for a in range(3):                           # bridge gaps (wheel arches) with a straight line, spray off
            pts[ii, a] = np.interp(ii, good, pts[good, a])
        stripe = []
        for i in ii:
            w = np.arange(max(first, i - smooth), min(last, i + smooth) + 1)
            n_avg = unit(nrm[w].sum(0))
            stripe.append({'p': pts[i].copy(), 'd': limit_tilt(-n_avg, approach, max_tilt),
                           'spray': bool(hit[i] and idx[i] < n_paint), 'stripe': k})
        if n_done % 2 == 1:                          # zig-zag: every other stripe runs backwards
            stripe.reverse()
        if path:                                     # step-over to the next stripe, spray off
            a, b = path[-1], stripe[0]
            n_link = max(1, int(np.linalg.norm(b['p'] - a['p']) / sample))
            for j in range(1, n_link):
                f = j / n_link
                path.append({'p': a['p'] * (1 - f) + b['p'] * f, 'd': unit(a['d'] * (1 - f) + b['d'] * f), 'spray': False, 'stripe': k})
        path += stripe
        n_done += 1
    return path
