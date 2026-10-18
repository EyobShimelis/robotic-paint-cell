"""Touch-up strokes for the spots the planned passes miss (simulated coverage), and approach / retract
points so the moves between passes start and end clear of the car."""
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from . import geometry as g


def simulate_coverage(centres, spray_points, radius):
    """Triangles (by centre) within the spray radius of any spray point."""
    painted = np.zeros(len(centres), dtype=bool)
    if len(spray_points):
        for hit in cKDTree(centres).query_ball_point(np.asarray(spray_points), radius):
            painted[hit] = True
    return painted


def missed_spots(centres, areas, painted, link=0.06, min_area=0.002):
    """Unpainted triangles grouped into spots (index arrays, largest first)."""
    idx = np.where(~painted)[0]
    if len(idx) == 0:
        return []
    pairs = cKDTree(centres[idx]).query_pairs(link, output_type='ndarray')
    graph = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(len(idx), len(idx)))
    n, label = connected_components(graph, directed=False)
    spots = [idx[label == k] for k in range(n)]
    spots = [s for s in spots if areas[s].sum() >= min_area]
    return sorted(spots, key=lambda s: -areas[s].sum())


def stroke(centres, normals, spot, approach, max_tilt, sample=0.05):
    """Short stroke along the spot's long axis, gun against the averaged surface normal."""
    c, n = centres[spot], normals[spot]
    mean = c.mean(0)
    axis = np.linalg.svd(c - mean, full_matrices=False)[2][0] if len(c) > 2 else np.eye(3)[0]
    t = (c - mean) @ axis
    stops = np.arange(t.min(), t.max() + 1e-9, sample)
    if len(stops) < 3:                              # small spot: 10 cm dab
        stops = np.array([-sample, 0.0, sample]) + (t.min() + t.max()) / 2
    path = []
    for s in stops:
        near = np.abs(t - s) <= sample
        if not near.any():
            near = np.abs(t - s) == np.abs(t - s).min()
        d = g.limit_tilt(-g.unit(n[near].sum(0)), approach, max_tilt)
        path.append({'p': c[near].mean(0), 'd': d, 'spray': True, 'stripe': -1})
    return path


def back_out(w, standoff=0.4, sample=0.05):
    """Points backing out along the gun axis, up to `standoff` from the surface."""
    return [{'p': w['p'] - w['d'] * o, 'd': w['d'], 'spray': False, 'stripe': -2}
            for o in np.arange(sample, standoff + 1e-9, sample)]


def with_approach_retract(path, standoff=0.4, sample=0.05):
    if not path:
        return path
    return back_out(path[0], standoff, sample)[::-1] + path + back_out(path[-1], standoff, sample)
