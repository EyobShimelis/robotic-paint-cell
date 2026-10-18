"""Figures for the results: coverage map and cycle-time chart (written next to paint_results.yaml).

They can also be redrawn from saved results:  python3 -m paint_cell_planner.report <results folder>
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                       # noqa: E402
import yaml                              # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402
from matplotlib.patches import Patch     # noqa: E402

PAINTED, MISSED = '#2a6fdb', '#e8413c'
# panel -> (horizontal axis, vertical axis, depth axis, viewer on the + side of the depth axis?), car frame
VIEWS = {'roof': (0, 1, 2, True, 'Roof (top view)'), 'hood': (0, 1, 2, True, 'Hood (top view)'),
         'left_side': (0, 2, 1, True, 'Left side'), 'right_side': (0, 2, 1, False, 'Right side'),
         'front': (1, 2, 0, True, 'Front'), 'rear': (1, 2, 0, False, 'Rear')}
PHASE_COLORS = {'paint': '#2a6fdb', 'touch-up': '#6aa7ff', 'approach / retract': '#f2b134',
                'travel (fold + track)': '#9a9a9a', 'conveyor': '#3aa86b'}


def coverage_map(npz_path, results, out_png):
    data = np.load(npz_path)
    panels = [z for z in VIEWS if f'{z}_tris' in data]
    cols = 3
    rows = int(np.ceil(len(panels) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(5.2 * cols, 3.6 * rows))
    for ax, zone in zip(np.ravel(axes), panels):
        tris, painted = data[f'{zone}_tris'], data[f'{zone}_painted']
        h, v, d, plus, title = VIEWS[zone]
        order = np.argsort(tris[:, :, d].mean(1) * (1 if plus else -1))       # far triangles first
        colors = np.where(painted[order], PAINTED, MISSED)
        ax.add_collection(PolyCollection(tris[order][:, :, [h, v]], facecolors=colors, edgecolors='none'))
        ax.autoscale()
        ax.set_aspect('equal')
        if not plus and h == 0:
            ax.invert_xaxis()                        # right side seen from the right
        cov = results.get('coverage', {}).get(zone, {}).get('coverage')
        ax.set_title(f'{title}: {100 * cov:.1f} % painted' if cov is not None else title, fontsize=11)
        ax.set_xlabel('m')
        ax.tick_params(labelsize=8)
    for ax in np.ravel(axes)[len(panels):]:
        ax.axis('off')
    total = results.get('coverage_total')
    fig.suptitle(f'Paint coverage map' + (f' - total {100 * total:.1f} %' if total is not None else ''), fontsize=14)
    fig.legend(handles=[Patch(color=PAINTED, label='painted'), Patch(color=MISSED, label='not painted')],
               loc='lower right', ncol=2)
    fig.tight_layout(rect=(0, 0.03, 1, 0.96))
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


def cycle_chart(events, out_png):
    """Timeline (Gantt) of the cycle and the total time per phase."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 5.5), gridspec_kw={'height_ratios': [1.1, 1]})
    labels = list(dict.fromkeys(e['label'] for e in events))
    for e in events:
        ax1.barh(labels.index(e['label']), e['end'] - e['start'], left=e['start'], height=0.6,
                 color=PHASE_COLORS.get(e['phase'], '#555'))
    ax1.set_yticks(range(len(labels)))
    ax1.set_yticklabels(labels, fontsize=8)
    ax1.invert_yaxis()
    ax1.set_xlabel('time (s)')
    ax1.set_title(f"Cycle timeline - {max(e['end'] for e in events):.0f} s in total")
    ax1.legend(handles=[Patch(color=c, label=p) for p, c in PHASE_COLORS.items()], fontsize=8, loc='lower right')
    totals = {p: sum(e['end'] - e['start'] for e in events if e['phase'] == p) for p in PHASE_COLORS}
    ax2.barh(list(totals), list(totals.values()), color=list(PHASE_COLORS.values()))
    for i, t in enumerate(totals.values()):
        ax2.text(t, i, f' {t:.0f} s', va='center', fontsize=9)
    ax2.invert_yaxis()
    ax2.set_xlabel('time (s)')
    ax2.set_title('Time per phase')
    fig.tight_layout()
    fig.savefig(out_png, dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    folder = sys.argv[1] if len(sys.argv) > 1 else 'paint_results'
    res = yaml.safe_load(open(os.path.join(folder, 'paint_results.yaml')))
    coverage_map(os.path.join(folder, 'coverage.npz'), res, os.path.join(folder, 'coverage_map.png'))
    if res.get('events'):
        cycle_chart(res['events'], os.path.join(folder, 'cycle_time.png'))
