#!/usr/bin/env python3
"""Generates the simple cell meshes (binary STL, metres): loop track, skid, conveyor, spray plume.

    python3 scripts/generate_meshes.py
"""
import math
import os
import struct

import numpy as np


def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    v = np.array([[x, y, z] for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)], dtype=float)
    f = [(0, 2, 1), (1, 2, 3), (4, 5, 6), (5, 7, 6), (0, 1, 4), (1, 5, 4),
         (2, 6, 3), (3, 6, 7), (0, 4, 2), (2, 4, 6), (1, 3, 5), (3, 7, 5)]
    return v[np.array(f)]


def cylinder_y(x, z, r, y0, y1, n=16):
    """Closed cylinder with its axis along Y (a conveyor roller)."""
    tris = []
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        p0 = (x + r * math.cos(a0), z + r * math.sin(a0)); p1 = (x + r * math.cos(a1), z + r * math.sin(a1))
        A, B = [p0[0], y0, p0[1]], [p1[0], y0, p1[1]]
        C, D = [p0[0], y1, p0[1]], [p1[0], y1, p1[1]]
        tris += [[A, C, B], [B, C, D], [[x, y0, z], A, B], [[x, y1, z], D, C]]
    return np.array(tris, dtype=float)


def write_stl(path, tris, name):
    t = np.concatenate(tris).astype('<f4')
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    rec = np.zeros(len(t), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    rec['n'], rec['v'] = n, t.reshape(-1, 9)
    with open(path, 'wb') as fh:
        fh.write(name.encode().ljust(80, b' ') + struct.pack('<I', len(rec)) + rec.tobytes())
    p = t.reshape(-1, 3)
    print(f'{os.path.relpath(path):38s} {len(t):5d} tris  min {np.round(p.min(0), 3)} max {np.round(p.max(0), 3)}')


# loop track: centreline x = +-4.0, y = +-2.5
def loop_track():
    X, Y = 4.0, 2.5
    B, R = (0.0, 0.02), (0.02, 0.10)            # base plate / rail heights
    b = []
    for s in (1, -1):
        b.append(box(-X - .15, X + .15, s * (Y - .15), s * (Y + .15), *B))        # base, long side
        b.append(box(s * (X - .15), s * (X + .15), -Y + .15, Y - .15, *B))        # base, end
        b.append(box(-X - .125, X + .125, s * (Y + .075), s * (Y + .125), *R))    # long outer rail
        b.append(box(-X + .075, X - .075, s * (Y - .125), s * (Y - .075), *R))    # long inner rail
        b.append(box(s * (X + .075), s * (X + .125), -Y - .125, Y + .125, *R))    # end outer rail
        b.append(box(s * (X - .125), s * (X - .075), -Y + .075, Y - .075, *R))    # end inner rail
    return b


# skid: origin = floor under its centre; car sits on the support rails at z = 0.50
SKID_L = 2.15            # half length
ROLLER_TOP = 0.40        # conveyor roller tops = underside of the skid runners
SUPPORT_X = (-0.6, 0.4)  # body supports under the sills, between the wheel arches
SILL_Z = 0.12            # underside of the car's sills above the car frame origin (skid top)


def skid():
    b = []
    for s in (1, -1):
        b.append(box(-SKID_L, SKID_L, s * 0.30, s * 0.40, ROLLER_TOP, 0.47))     # runner riding on the rollers
        b.append(box(-SKID_L, SKID_L, s * 0.62, s * 0.70, 0.44, 0.50))           # car support rail
    for x in (-2.0, -1.0, 0.0, 1.0, 2.0):
        b.append(box(x - 0.04, x + 0.04, -0.70, 0.70, 0.42, 0.47))               # cross-member
    # the car has no floor: it rests on its sills (y = +-0.9) on four posts
    for x in SUPPORT_X:
        for s in (1, -1):
            b.append(box(x - 0.05, x + 0.05, s * 0.62, s * 0.94, 0.44, 0.50))    # outrigger
            b.append(box(x - 0.05, x + 0.05, s * 0.84, s * 0.94, 0.50, 0.50 + SILL_Z))   # post under the sill
    return b


# conveyor: along X at y = 0, three sections
CONV_END = 12.3          # reaches 5 m outside the building so the car can wait fully outside
CONV_GAP = 0.7           # half length of the transfer gap at each robot-track crossing (x = +-4.0)
CONV_SECTIONS = [(-CONV_END, -4.0 - CONV_GAP), (-4.0 + CONV_GAP, 4.0 - CONV_GAP), (4.0 + CONV_GAP, CONV_END)]
ROLLER_R = 0.045


def conveyor():
    frame, rollers = [], []
    for xa, xb in CONV_SECTIONS:
        for s in (1, -1):
            frame.append(box(xa, xb, s * 0.44, s * 0.50, -0.02, 0.34))
        n = int((xb - xa - 0.1) / 0.25) + 1
        for i in range(n):
            x = xa + 0.05 + (xb - xa - 0.1) * i / (n - 1)
            rollers.append(cylinder_y(x, ROLLER_TOP - ROLLER_R, ROLLER_R, -0.44, 0.44))
    return frame, rollers


# spray plume: cone from the nozzle to the paint surface, in the spray gun frame
NOZZLE_Z, SURFACE_Z, PLUME_R = 0.35, 0.595, 0.11      # nozzle tip, just short of the spray point, spray radius


def plume(n=32):
    tris = []
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        p0 = [PLUME_R * math.cos(a0), PLUME_R * math.sin(a0), SURFACE_Z]
        p1 = [PLUME_R * math.cos(a1), PLUME_R * math.sin(a1), SURFACE_Z]
        tris += [[[0, 0, NOZZLE_Z], p0, p1], [[0, 0, NOZZLE_Z], p1, p0]]
    return [np.array(tris, dtype=float)]


if __name__ == '__main__':
    pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    vis, col = os.path.join(pkg, 'meshes', 'visual'), os.path.join(pkg, 'meshes', 'collision')
    for d in (vis, col):
        write_stl(os.path.join(d, 'loop_track.stl'), loop_track(), 'loop track with L corners')
        write_stl(os.path.join(d, 'skid.stl'), skid(), 'conveyor skid')
    frame, rollers = conveyor()
    write_stl(os.path.join(vis, 'conveyor_frame.stl'), frame, 'roller conveyor frame')
    write_stl(os.path.join(vis, 'conveyor_rollers.stl'), rollers, 'roller conveyor rollers')
    write_stl(os.path.join(vis, 'spray_plume.stl'), plume(), 'spray plume')
    print('conveyor sections (x from, x to):', CONV_SECTIONS)
