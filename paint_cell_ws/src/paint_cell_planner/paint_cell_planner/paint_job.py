"""Paint job: plans the paint passes and touch-ups for each zone, then runs the cycle
(car in -> paint -> car out) and writes the results.

    ros2 run paint_cell_planner paint_job --ros-args -p gazebo:=true
"""
import os
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial import cKDTree
from tf2_ros import Buffer, TransformListener

from . import geometry as g
from . import report, touchup
from .moveit_io import ARM, TRACK, MoveItIO
from .paint_fx import PaintFX

# rail -> fixed track coordinate and the joint_1 angle that faces the car
RAILS = {'near': {'track_y': 0.0, 'j1': 0.0}, 'far': {'track_y': 5.0, 'j1': np.pi},
         'front': {'track_x': 4.0, 'j1': -np.pi / 2}, 'rear': {'track_x': -4.0, 'j1': np.pi / 2}}
TRAVEL = [-np.pi / 2, 2.6, 0.0, 1.5, 0.0]     # joint_2..6, folded; clears the walls everywhere on the loop
HOME = [0.0] + TRAVEL
V_MAX = {'track_x': 1.0, 'track_y': 1.0, 'joint_1': 1.83, 'joint_2': 1.64, 'joint_3': 1.74, 'joint_4': 2.37,
         'joint_5': 2.25, 'joint_6': 3.59}
A_MAX = {'track_x': 0.5, 'track_y': 0.5, 'joint_1': 2.0, 'joint_2': 2.0, 'joint_3': 2.0, 'joint_4': 3.0,
         'joint_5': 3.0, 'joint_6': 3.0}


def rail_of(x, y):
    if abs(y) < 1e-3:
        return 'near'
    if abs(y - 5.0) < 1e-3:
        return 'far'
    return 'front' if x > 0 else 'rear'


def route_position(x, y):
    """Position along the route far -> rear -> near -> front (0 at the near-rear corner).
    Not a closed loop: joint 1 is limited to +-185 deg and can't turn towards the walls, so the robot
    never passes the far-front corner."""
    return {'near': x + 4.0, 'rear': -y, 'far': -5.0 - (x + 4.0), 'front': 8.0 + y}[rail_of(x, y)]


def loop_route(start, goal):
    s0, s1 = route_position(*start), route_position(*goal)
    corners = [(8.0, (4.0, 0.0)), (0.0, (-4.0, 0.0)), (-5.0, (-4.0, 5.0))]
    between = [(abs(s - s0), c) for s, c in corners if min(s0, s1) + 1e-6 < s < max(s0, s1) - 1e-6]
    return [c for _, c in sorted(between)] + [tuple(goal)]


def leg_rail(a, b):
    if abs(a[1]) < 1e-3 and abs(b[1]) < 1e-3:
        return 'near'
    if abs(a[1] - 5.0) < 1e-3 and abs(b[1] - 5.0) < 1e-3:
        return 'far'
    return 'front' if a[0] > 0 else 'rear'


def time_parameterize(names, q, tcp, speed):
    """Constant TCP speed, stretched where a joint velocity / acceleration limit would be exceeded."""
    q = np.asarray(q)
    vmax = np.array([V_MAX[n] for n in names]) * 0.9
    amax = np.array([A_MAX[n] for n in names])
    dq = np.diff(q, axis=0)
    dt = np.maximum(np.linalg.norm(np.diff(tcp, axis=0), axis=1) / speed, 1e-3)
    dt = np.maximum(dt, (np.abs(dq) / vmax).max(1))
    for _ in range(400):
        v = np.vstack([np.zeros(len(names)), dq / dt[:, None], np.zeros(len(names))])
        h = np.concatenate([[dt[0] / 2], (dt[:-1] + dt[1:]) / 2, [dt[-1] / 2]])
        over = (np.abs(np.diff(v, axis=0)) / h[:, None] / amax).max(1)
        if over.max() <= 1.0:
            break
        scale = np.ones(len(dt))
        slow = np.where(over > 1.0)[0]
        for i in slow:
            for j in (i - 1, i):
                if 0 <= j < len(dt):
                    scale[j] = max(scale[j], min(1.08, over[i] ** 0.5))
        dt *= scale
    seg_v = dq / dt[:, None]
    vel = np.vstack([np.zeros(len(names)), (seg_v[:-1] + seg_v[1:]) / 2, np.zeros(len(names))])
    return np.concatenate([[0.0], np.cumsum(dt)]), vel


class PaintJob(Node):
    def __init__(self):
        super().__init__('paint_job')
        share = get_package_share_directory('paint_cell_planner')
        p = self.declare_parameter
        self.cfg = yaml.safe_load(open(p('config', os.path.join(share, 'config', 'paint_job.yaml')).value))
        self.zones = list(p('zones', ['all']).value)
        self.gazebo = p('gazebo', False).value
        self.execute = p('execute', True).value                  # false: plan only
        self.move_car = p('move_car', True).value
        self.touchups = p('touchups', True).value
        self.output_dir = p('output_dir', 'paint_results').value
        self.mesh_dir = os.path.join(get_package_share_directory('paint_cell_description'), 'meshes')
        self.io = MoveItIO(self)
        self.io.transit_scale = float(self.cfg['paint'].get('transit_speed_scale', 1.0))
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.car_offset = np.array([0.0, 0.0, self.cfg['car']['bottom_z']])
        paint = self.cfg['paint']
        all_zones = list(dict.fromkeys(j['zone'] for j in self.cfg['jobs']))
        self.fx = PaintFX(self, os.path.join(self.mesh_dir, 'visual'), all_zones, paint['color'], paint['spray_radius'],
                          gazebo=self.gazebo, world=self.cfg['gazebo_world'], car_z=self.cfg['car']['bottom_z'])
        self.results, self.events, self.planned = [], [], {}
        self.retracted_on = None
        self.t_start = None

    def _half(self, name):
        t = g.load_stl(os.path.join(self.mesh_dir, 'visual', name + '.stl'))
        return t[:len(t) // 2]

    def main_path(self, job):
        # glass and trim only carry the path across the openings (spray off)
        paint = self.cfg['paint']
        surface = [self._half('paint_' + job['zone'])]
        zone_file = os.path.join(self.mesh_dir, 'zones', job['zone'] + '.stl')
        if os.path.exists(zone_file):
            tris = g.orient_outward(g.load_stl(zone_file))
            side = np.sign(job['approach'][1])
            surface.append(tris[tris.mean(1)[:, 1] * side < 0] if side else tris)
        n_paint = sum(len(t) for t in surface)
        tris = np.concatenate(surface + [g.orient_outward(self._half('car_glass')), g.orient_outward(self._half('car_trim'))])
        return g.raster_path(tris, job['approach'], job['stripe_axis'], job['step_axis'], spacing=paint['stripe_spacing'],
                             sample=paint['sample'], max_tilt=np.radians(paint['max_tilt_deg']),
                             step_range=job.get('step_range'), n_paint=n_paint)

    def plan_touchups(self, jobs, paths):
        """Strokes for the spots the planned passes miss: {job index: [stroke path, ...]}.
        Each spot goes to the job whose spray direction faces it best."""
        paint = self.cfg['paint']
        radius, max_tilt = paint['spray_radius'], np.radians(paint['touchup_max_tilt_deg'])
        strokes = {}
        for zone in dict.fromkeys(j['zone'] for j in jobs):
            panel = self.fx.panels[zone]
            idx = [i for i, j in enumerate(jobs) if j['zone'] == zone]
            sprayed = [w['p'] for i in idx for w in paths[i] if w['spray']]
            painted = touchup.simulate_coverage(panel['centre'], sprayed, radius)
            spots = touchup.missed_spots(panel['centre'], panel['area'], painted)
            touched = []
            for spot in spots:
                centre, normal = panel['centre'][spot].mean(0), g.unit(panel['normal'][spot].sum(0))

                def fits(i):
                    r = jobs[i].get('step_range')
                    return r is None or r[0] - 0.1 <= centre[g.AXIS[jobs[i]['step_axis']]] <= r[1] + 0.1
                best = max((i for i in idx if fits(i)), key=lambda i: g.unit(jobs[i]['approach']) @ -normal, default=None)
                if best is None:
                    continue
                st = touchup.stroke(panel['centre'], panel['normal'], spot, g.unit(jobs[best]['approach']), max_tilt,
                                    paint['sample'])
                strokes.setdefault(best, []).append(st)
                touched += [w['p'] for w in st]
            after = touchup.simulate_coverage(panel['centre'], sprayed + touched, radius)
            area = panel['area'].sum()
            self.planned[zone] = {'passes': round(float(panel['area'][painted].sum() / area), 4),
                                  'with_touchups': round(float(panel['area'][after].sum() / area), 4),
                                  'missed_spots': len(spots)}
        return {i: sorted(s, key=lambda st: np.mean([w['p'] for w in st], axis=0)[0 if jobs[i]['rail'] in ('near', 'far') else 1])
                for i, s in strokes.items()}

    def plan_job(self, job, path, kind='paint'):
        """Path (car frame) -> joint trajectory. The track follows the gun; IK is seeded with the previous point."""
        paint = self.cfg['paint']
        zone, rail = job['zone'], job['rail']
        n_strokes = len({w['stripe'] for w in path}) if kind == 'paint' else sum(
            1 for a, b in zip([{'spray': False}] + path, path) if b['spray'] and not a['spray'])
        path = touchup.with_approach_retract(path, paint['retract'], paint['sample'])
        # gun roll is free (round spray); avoid a reference axis close to the gun axis or the wrist spins
        stripe_dir = np.eye(3)[g.AXIS[job['stripe_axis']]]
        if max((abs(w['d'] @ stripe_dir) for w in path), default=0.0) > 0.7:
            stripe_dir = np.eye(3)[g.AXIS[job['step_axis']]]
        lead = job.get('lead', 0.0)
        best = None
        for seed in ([-1.3, 1.5, 0.0, 1.2, 0.0], [-1.0, 1.2, 0.0, 0.8, 0.0], [-1.6, 1.9, 0.0, 1.3, 0.0]):
            q = [RAILS[rail]['j1']] + seed
            solved, n_spray, n_ok = [], 0, 0
            for w in path:
                pw = w['p'] + self.car_offset
                if rail in ('near', 'far'):
                    track = [float(np.clip(pw[0] + lead, -4.0, 4.0)), RAILS[rail]['track_y']]
                else:
                    track = [RAILS[rail]['track_x'], float(np.clip(pw[1] + 2.5 + lead, 0.0, 5.0))]
                quat = g.quaternion_from_matrix(g.rotation_from_tool_axes(w['d'], stripe_dir))
                sol = None
                for _ in range(3):
                    s = self.io.solve_ik(pw, quat, {'conveyor_x': 0.0, **dict(zip(TRACK, track)), **dict(zip(ARM, q))})
                    if s is not None:
                        # wrist joints: take the 2*pi-equivalent closest to the previous point (avoids the end stops)
                        for k in (3, 5):
                            s[k] = q[k] + (s[k] - q[k] + np.pi) % (2 * np.pi) - np.pi
                    if s is not None and (not solved or np.max(np.abs(np.array(s) - np.array(q))) < 0.6):
                        sol = s
                        break
                n_spray += w['spray']
                if sol is not None:
                    q = sol
                    n_ok += w['spray']
                    solved.append({'q': track + sol, 'p': w['p'], 'spray': w['spray']})
            frac = n_ok / max(1, n_spray)
            if best is None or frac > best['fraction']:
                best = {'fraction': frac, 'solved': solved}
            if frac > 0.995:
                break
        solved = best['solved']
        names = TRACK + ARM
        if len(solved) < 2:
            return {'job': job, 'kind': kind, 'names': names, 'q': np.zeros((0, 8)),
                    'stats': {'zone': zone, 'rail': rail, 'kind': kind, 'strokes': n_strokes, 'waypoints': len(path),
                              'ik_fraction': 0.0, 'path_length_m': 0.0, 'spray_length_m': 0.0, 'max_tilt_deg': 0.0,
                              'planned_time_s': 0.0, 'wrist_range_rad': [0.0, 0.0, 0.0, 0.0]}}
        q = np.array([s['q'] for s in solved])
        pts = np.array([s['p'] for s in solved])
        spray = np.array([s['spray'] for s in solved])
        times, vel = time_parameterize(names, q, pts, paint['speed'])
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        dirs = np.array([w['d'] for w in path if w['spray']])
        tilt = np.degrees(np.arccos(np.clip(dirs @ g.unit(job['approach']), -1, 1))) if len(dirs) else np.zeros(1)
        return {'job': job, 'kind': kind, 'names': names, 'q': q, 'times': times, 'vel': vel, 'points': pts, 'spray': spray,
                'stats': {'zone': zone, 'rail': rail, 'kind': kind, 'strokes': n_strokes, 'waypoints': len(path),
                          'ik_fraction': round(best['fraction'], 4), 'path_length_m': round(float(seg.sum()), 2),
                          'spray_length_m': round(float(seg[spray[1:] & spray[:-1]].sum()), 2),
                          'max_tilt_deg': round(float(tilt.max()), 1), 'planned_time_s': round(float(times[-1]), 1),
                          'wrist_range_rad': [round(float(q[:, 5].min()), 2), round(float(q[:, 5].max()), 2),
                                              round(float(q[:, 7].min()), 2), round(float(q[:, 7].max()), 2)]}}

    def must(self, ok, what):
        if not ok:
            raise RuntimeError(f'motion failed: {what}')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def record(self, phase, label, start):
        self.events.append({'phase': phase, 'label': label, 'start': round(start - self.t_start, 2),
                            'end': round(self.now() - self.t_start, 2)})

    def track_now(self):
        x, y = self.io.current(TRACK)
        x = min((-4.0, 4.0), key=lambda c: abs(c - x)) if min(abs(x + 4), abs(x - 4)) < 0.02 else x
        y = min((0.0, 5.0), key=lambda c: abs(c - y)) if min(abs(y), abs(y - 5)) < 0.02 else y
        return x, y

    def settle(self, names, target, tolerance=0.005, timeout=60.0):
        # simulated joints lag; MoveIt refuses to start if the robot is not at the trajectory start
        end = time.time() + timeout
        while time.time() < end:
            if np.max(np.abs(np.array(self.io.current(names)) - np.asarray(target))) < tolerance:
                return
        self.get_logger().warn('joints did not settle at the start of the paint pass')

    def tuck(self, rail):
        self.must(self.io.move_joints('arm', dict(zip(ARM, [RAILS[rail]['j1']] + TRAVEL))), f'tuck arm ({rail} rail)')
        self.retracted_on = None

    def drive_to(self, goal, label):
        start, t0 = self.track_now(), self.now()
        if np.hypot(start[0] - goal[0], start[1] - goal[1]) >= 1e-3:
            self.tuck(rail_of(*start))
            for x, y in loop_route(start, goal):
                rail = leg_rail(self.track_now(), (x, y))
                if abs(self.io.current(['joint_1'])[0] - RAILS[rail]['j1']) > 0.05:
                    self.tuck(rail)
                self.must(self.io.move_joints('track', {'track_x': x, 'track_y': y}), f'track to ({x:.1f}, {y:.1f})')
        self.record('travel (fold + track)', label, t0)

    def go_to_start(self, plan, label):
        # same side as the last pass: try a direct move first, otherwise fold, drive and unfold
        q0 = plan['q'][0]
        if self.retracted_on == plan['job']['rail']:
            t0 = self.now()
            if self.io.move_joints('arm_track', dict(zip(plan['names'], q0)), straight_only=True):
                self.record('approach / retract', label, t0)
                return
        self.drive_to((float(q0[0]), float(q0[1])), label)
        t0 = self.now()
        self.tuck(plan['job']['rail'])
        self.must(self.io.move_joints('arm', dict(zip(ARM, q0[2:]))), f"approach {plan['job']['zone']}")
        self.record('approach / retract', label, t0)

    def paint(self, plan):
        job, zone = plan['job'], plan['job']['zone']
        label = f"{zone} ({job['rail']})" + (' touch-up' if plan['kind'] == 'touch-up' else '')
        self.go_to_start(plan, label)
        self.settle(plan['names'], plan['q'][0])
        tree = cKDTree(plan['points'])
        state = {'last_flush': time.time(), 'last_sample': 0.0, 'last_spray': 0.0}

        def on_spin():
            now = time.time()
            if now - state['last_sample'] < 0.05:
                return
            state['last_sample'] = now
            try:
                t = self.tf.lookup_transform('car_body', 'paint_tcp', Time()).transform.translation
            except Exception:
                return
            p = np.array([t.x, t.y, t.z])
            d, i = tree.query(p)
            spraying = bool(d < 0.08 and plan['spray'][i])
            if spraying:
                state['last_spray'] = now
            self.fx.plume(now - state['last_spray'] < 0.6)
            if spraying:
                self.fx.spray(zone, p)
            else:
                self.fx.lift()
            if now - state['last_flush'] > self.cfg['paint']['flush_period']:
                self.fx.flush(zone)
                state['last_flush'] = now

        t0 = self.now()
        future = self.io.execute_trajectory(plan['names'], plan['q'], plan['times'], plan['vel'])
        ok = self.io.wait_result(future, on_spin)
        self.fx.plume(False)
        self.fx.lift()
        self.fx.flush(zone)
        self.must(ok, f'paint {zone}')
        self.record('touch-up' if plan['kind'] == 'touch-up' else 'paint', label, t0)
        plan['stats']['paint_time_s'] = round(self.now() - t0, 1)
        self.retracted_on = job['rail']

    def conveyor(self, target, label):
        t0 = self.now()

        def follow():
            self.fx.follow_car(self.io.joints.get('conveyor_x', 0.0), self.io.velocities.get('conveyor_x', 0.0))
        self.must(self.io.move_joints('conveyor', {'conveyor_x': target}, velocity=1.0, acceleration=1.0, on_spin=follow),
                  label)
        self.fx.follow_car(self.io.current(['conveyor_x'])[0])
        self.record('conveyor', label, t0)

    @staticmethod
    def touchup_stats(job, done, planned):
        return {'zone': job['zone'], 'rail': job['rail'], 'kind': 'touch-up', 'strokes': len(done),
                'strokes_planned': planned, 'waypoints': sum(p['stats']['waypoints'] for p in done),
                'ik_fraction': round(len(done) / max(1, planned), 4),
                'path_length_m': round(sum(p['stats']['path_length_m'] for p in done), 2),
                'spray_length_m': round(sum(p['stats']['spray_length_m'] for p in done), 2),
                'max_tilt_deg': max([p['stats']['max_tilt_deg'] for p in done], default=0.0),
                'planned_time_s': round(sum(p['stats']['planned_time_s'] for p in done), 1),
                'wrist_range_rad': [min([p['stats']['wrist_range_rad'][0] for p in done], default=0.0),
                                    max([p['stats']['wrist_range_rad'][1] for p in done], default=0.0),
                                    min([p['stats']['wrist_range_rad'][2] for p in done], default=0.0),
                                    max([p['stats']['wrist_range_rad'][3] for p in done], default=0.0)]}

    def run(self):
        log = self.get_logger()
        self.io.wait_ready()
        jobs = [j for j in self.cfg['jobs'] if 'all' in self.zones or j['zone'] in self.zones]
        zones = list(dict.fromkeys(j['zone'] for j in jobs))

        log.info('planning the paint paths ...')
        paths = [self.main_path(j) for j in jobs]
        extra = self.plan_touchups(jobs, paths) if self.touchups else {}
        for zone, c in self.planned.items():
            log.info(f"  {zone}: passes cover {100 * c['passes']:.1f} %, {c['missed_spots']} missed spots -> "
                     f"{100 * c['with_touchups']:.1f} % with touch-ups (planned)")

        steps = []                                   # plan None = panel finished
        for n, job in enumerate(jobs):
            log.info(f"planning {job['zone']} from the {job['rail']} rail ...")
            plan = self.plan_job(job, paths[n])
            s = plan['stats']
            log.info(f"  {s['strokes']} stripes, {s['spray_length_m']} m of spray, IK solved for "
                     f"{100 * s['ik_fraction']:.1f} % of the spray points, {s['planned_time_s']} s")
            steps.append((job, plan))
            if extra.get(n):
                touch = [self.plan_job(job, st, 'touch-up') for st in extra[n]]
                done = [p for p in touch if p['stats']['ik_fraction'] >= 0.5 and len(p['q']) > 1]
                log.info(f"  touch-up: {len(done)} of {len(touch)} strokes reachable, "
                         f"{sum(p['stats']['spray_length_m'] for p in done):.2f} m of spray")
                steps += [(job, p) for p in done]
                self.results.append(self.touchup_stats(job, done, len(touch)))
            if all(later['zone'] != job['zone'] for later in jobs[n + 1:]):
                steps.append((job, None))
        self.t_start = self.now()

        if self.execute and self.move_car and abs(self.io.current(['conveyor_x'])[0]) > 1e-3:
            log.info('car enters on the conveyor')
            self.conveyor(0.0, 'car in')

        if self.execute:
            for job, plan in steps:
                if plan is None:
                    self.fx.finish_zone(job['zone'])
                    continue
                self.paint(plan)
                plan['stats']['zone_coverage_after'] = round(self.fx.coverage(job['zone']), 4)
                log.info(f"  {plan['kind']} {job['zone']} ({job['rail']}) done; coverage now "
                         f"{100 * plan['stats']['zone_coverage_after']:.1f} %")
        for job, plan in steps:
            if plan is not None and plan['kind'] == 'paint':
                self.results.append(plan['stats'])
        for row in self.results:
            if row['kind'] == 'touch-up':
                runs = [p['stats'] for j, p in steps if p is not None and p['kind'] == 'touch-up'
                        and j['zone'] == row['zone'] and j['rail'] == row['rail']]
                if self.execute:
                    row['paint_time_s'] = round(sum(r.get('paint_time_s', 0.0) for r in runs), 1)
        order = {(j['zone'], j['rail']): i for i, j in enumerate(jobs)}
        self.results.sort(key=lambda r: (order[(r['zone'], r['rail'])], r['kind'] != 'paint'))

        if self.execute:
            log.info('robot returns home')
            self.drive_to((0.0, 0.0), 'home')
            t0 = self.now()
            self.must(self.io.move_joints('arm', dict(zip(ARM, HOME))), 'home')
            self.record('travel (fold + track)', 'home', t0)
            if self.move_car:
                log.info('car leaves on the conveyor')
                self.fx.prepare_exit()
                self.conveyor(self.cfg['conveyor']['outside_exit'], 'car out')
        self.fx.wait_cleanup()
        self.write_results(zones, self.now() - self.t_start)

    def write_results(self, zones, total_time):
        os.makedirs(self.output_dir, exist_ok=True)
        summary = {'executed': bool(self.execute), 'simulator': 'gazebo' if self.gazebo else 'mock hardware',
                   'total_time_s': round(total_time, 1), 'paint': self.cfg['paint'], 'planned_coverage': self.planned,
                   'jobs': self.results}
        if self.execute:
            area = {z: float(self.fx.panels[z]['area'].sum()) for z in zones}
            summary['coverage'] = {z: {'panel_area_m2': round(area[z], 2), 'painted_area_m2': round(self.fx.painted_area(z), 2),
                                       'coverage': round(self.fx.coverage(z), 4)} for z in zones}
            summary['coverage_total'] = round(sum(self.fx.painted_area(z) for z in zones) / sum(area.values()), 4)
            summary['events'] = self.events
        with open(os.path.join(self.output_dir, 'paint_results.yaml'), 'w') as fh:
            yaml.safe_dump(summary, fh, sort_keys=False)
        lines = ['| Zone | Rail | Pass | Strokes | Spray length (m) | IK solved | Max tilt (deg) | Time (s) |',
                 '|---|---|---|---|---|---|---|---|']
        for s in self.results:
            lines.append(f"| {s['zone']} | {s['rail']} | {s['kind']} | {s['strokes']} | {s['spray_length_m']} | "
                         f"{100 * s['ik_fraction']:.1f} % | {s['max_tilt_deg']} | {s.get('paint_time_s', s['planned_time_s'])} |")
        if self.execute:
            lines += ['', '| Panel | Area (m2) | Painted (m2) | Coverage |', '|---|---|---|---|']
            for z, c in summary['coverage'].items():
                lines.append(f"| {z} | {c['panel_area_m2']} | {c['painted_area_m2']} | {100 * c['coverage']:.1f} % |")
            lines.append(f"| **total** | | | **{100 * summary['coverage_total']:.1f} %** |")
            lines.append(f"\nCycle time: {total_time:.0f} s")
        with open(os.path.join(self.output_dir, 'paint_results.md'), 'w') as fh:
            fh.write('\n'.join(lines) + '\n')
        if self.execute:
            npz = os.path.join(self.output_dir, 'coverage.npz')
            self.fx.save_coverage(npz)
            report.coverage_map(npz, summary, os.path.join(self.output_dir, 'coverage_map.png'))
            report.cycle_chart(self.events, os.path.join(self.output_dir, 'cycle_time.png'))
        self.get_logger().info('results:\n' + '\n'.join(lines))
        self.get_logger().info(f'saved to {os.path.abspath(self.output_dir)}')


def main():
    rclpy.init()
    node = PaintJob()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
