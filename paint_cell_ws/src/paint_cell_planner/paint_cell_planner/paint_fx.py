"""Paint effect and coverage: panel triangles within the spray radius of the measured spray point are
marked painted, shown as RViz markers and as thin patch models in Gazebo, and counted for the coverage."""
import os
import queue
import re
import subprocess
import tempfile
import threading
import time

import numpy as np
from geometry_msgs.msg import Point
from rclpy.qos import DurabilityPolicy, QoSProfile
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from scipy.spatial import cKDTree
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from . import geometry as g

OFFSET = 0.003          # patches sit 3 mm above the car surface
PATCH_EDGE = 0.05       # subdivide panels to this edge length -> smooth stripe edges
MERGE_EVERY = 6         # many small models slow Gazebo down
PLUME_MESH = 'package://paint_cell_description/meshes/visual/spray_plume.stl'


class PaintFX:
    def __init__(self, node, mesh_dir, zones, color, spray_radius, gazebo=False, world='paint_booth', car_z=0.5):
        self.node, self.color, self.radius = node, list(color), spray_radius
        self.gazebo, self.world, self.car_z = gazebo, world, car_z
        self.panels = {}
        for i, zone in enumerate(zones):
            tris = g.load_stl(os.path.join(mesh_dir, f'paint_{zone}.stl'))
            tris = tris[:len(tris) // 2]  # first half: outward-facing copy
            tris = g.subdivide(tris, PATCH_EDGE)
            self.panels[zone] = {
                'id': i, 'tris': tris, 'centre': tris.mean(1), 'tree': cKDTree(tris.mean(1)),
                'normal': g.triangle_normals(tris), 'area': g.triangle_areas(tris),
                'painted': np.zeros(len(tris), dtype=bool), 'pending': np.zeros(len(tris), dtype=bool),
            }
        self.marker_pub = node.create_publisher(
            Marker, '/paint_markers', QoSProfile(depth=2000, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tmp = tempfile.mkdtemp(prefix='paint_patches_')
        self.live = {z: [] for z in zones}
        self.retiring = []          # (time, names) waiting to be removed
        self.patch_count = 0
        self.car_x = 0.0
        self.remove_queue = queue.Queue()
        threading.Thread(target=self._remove_worker, daemon=True).start()
        self.last_point = None
        self.marker_count = 0
        self.plume_on = None
        self.plume_id = None
        self.last_follow = 0.0
        # via ros_gz_bridge: a `gz service` call per pose update is too slow
        self.set_pose = node.create_client(SetEntityPose, f'/world/{world}/set_pose')
        self.pose_future = None

    def lift(self):
        self.last_point = None

    def spray(self, zone, point):
        """Mark triangles within the spray radius of the segment from the previous sample to `point` (car frame)."""
        p = self.panels[zone]
        a = point if self.last_point is None else self.last_point
        b = np.asarray(point, dtype=float)
        self.last_point = b
        seg = b - a
        length = np.linalg.norm(seg)
        idx = np.array(p['tree'].query_ball_point((a + b) / 2, self.radius + length / 2), dtype=int)
        if len(idx) == 0:
            return
        c = p['centre'][idx]
        f = np.clip(((c - a) @ seg) / (length ** 2 + 1e-12), 0, 1)
        hit = idx[np.linalg.norm(c - (a + f[:, None] * seg), axis=1) <= self.radius]
        new = hit[~p['painted'][hit]]
        p['painted'][new] = True
        p['pending'][new] = True

    def coverage(self, zone):
        p = self.panels[zone]
        return float(p['area'][p['painted']].sum() / p['area'].sum())

    def painted_area(self, zone):
        p = self.panels[zone]
        return float(p['area'][p['painted']].sum())

    def save_coverage(self, path):
        data = {}
        for z, p in self.panels.items():
            data[f'{z}_tris'], data[f'{z}_painted'] = p['tris'].astype(np.float32), p['painted']
        np.savez_compressed(path, **data)

    def flush(self, zone):
        self._retire_due()
        p = self.panels[zone]
        new = np.where(p['pending'])[0]
        if len(new) == 0:
            return
        p['pending'][:] = False
        self._publish_marker(zone, new)
        if not self.gazebo:
            return
        self.live[zone].append(self._spawn_patch(zone, new))
        if len(self.live[zone]) >= MERGE_EVERY:
            self._merge(zone)

    def _merge(self, zone):
        # old patches are removed a few seconds later, once the merged one is on screen
        old, self.live[zone] = self.live[zone], [self._spawn_patch(zone, np.where(self.panels[zone]['painted'])[0])]
        self.retiring.append((time.time() + 3.0, old))

    def finish_zone(self, zone):
        self.flush(zone)
        if self.gazebo and len(self.live[zone]) > 1:
            self._merge(zone)

    def prepare_exit(self):
        # one model for all paint, so a single pose update moves it with the car
        for zone in self.panels:
            self.flush(zone)
        if not self.gazebo:
            return
        tris = np.concatenate([p['tris'][p['painted']] + OFFSET * p['normal'][p['painted']][:, None, :]
                               for p in self.panels.values()])
        if len(tris) == 0:
            return
        old = [n for z in self.live for n in self.live[z]]
        self.live = {z: [] for z in self.live}
        self.live['car'] = [self._spawn_tris('paint_car', tris)]
        time.sleep(3.0)
        self._retire(old + [n for _, names in self.retiring for n in names])
        self.retiring = []
        self.set_pose.wait_for_service(timeout_sec=5.0)

    def follow_car(self, car_x, car_v=0.0, latency=0.05):
        # position predicted slightly ahead to make up for the update delay
        if not self.gazebo or time.time() - self.last_follow < 0.04 or 'car' not in self.live:
            return
        if self.pose_future is not None and not self.pose_future.done():
            return
        self.last_follow = time.time()
        self.car_x = car_x + car_v * latency
        req = SetEntityPose.Request()
        req.entity = Entity(name=self.live['car'][0], type=Entity.MODEL)
        req.pose.position.x, req.pose.position.z = float(self.car_x), float(self.car_z)
        req.pose.orientation.w = 1.0
        self.pose_future = self.set_pose.call_async(req)

    def plume(self, on):
        if on == self.plume_on:
            return
        self.plume_on = on
        m = Marker()
        m.header.frame_id = 'spray_gun'
        m.ns, m.id, m.type = 'spray_plume', 0, Marker.MESH_RESOURCE
        m.action = Marker.ADD if on else Marker.DELETE
        m.mesh_resource = PLUME_MESH
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color = ColorRGBA(r=0.35, g=0.55, b=1.0, a=0.45)
        self.marker_pub.publish(m)
        if not self.gazebo:
            return
        if self.plume_id is None:
            self.plume_id = self._visual_id('spray_plume')
            if self.plume_id is None:
                return
        a, tr = (0.7, 0.3) if on else (0.0, 1.0)
        col = f'{{r: 0.2 g: 0.45 b: 1.0 a: {a}}}'
        emissive = '{r: 0.15 g: 0.3 b: 0.8 a: 1}' if on else '{r: 0 g: 0 b: 0 a: 0}'
        self._gz('visual_config', 'gz.msgs.Visual',
                 f'id: {self.plume_id} transparency: {tr} material {{ambient {col} diffuse {col} emissive {emissive}}}')

    def _publish_marker(self, zone, index):
        p = self.panels[zone]
        m = Marker()
        m.header.frame_id = 'car_body'
        self.marker_count += 1
        m.ns, m.id, m.type, m.action = 'paint_' + zone, self.marker_count, Marker.TRIANGLE_LIST, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color = ColorRGBA(r=self.color[0], g=self.color[1], b=self.color[2], a=1.0)
        tris = p['tris'][index] + OFFSET * p['normal'][index][:, None, :]
        both = np.concatenate([tris, tris[:, [0, 2, 1]]]).reshape(-1, 3)
        m.points = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in both]
        self.marker_pub.publish(m)

    def _gz(self, service, reqtype, req, wait=False, reptype='gz.msgs.Boolean'):
        cmd = ['gz', 'service', '-s', f'/world/{self.world}/{service}', '--reqtype', reqtype,
               '--reptype', reptype, '--timeout', '5000', '--req', req]
        if wait:
            try:
                return subprocess.run(cmd, capture_output=True, text=True, timeout=30).stdout
            except subprocess.TimeoutExpired:
                return ''
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return None

    def _visual_id(self, mesh_name):
        # visual_config only works with the entity id; retried because Gazebo can be slow to answer
        for _ in range(4):
            scene = self._gz('scene/info', 'gz.msgs.Empty', '', wait=True, reptype='gz.msgs.Scene')
            m = re.search(r'visual \{\s*name: "[^"]*"\s*id: (\d+)(?:(?!visual \{).)*?%s\.stl' % mesh_name, scene, re.S)
            if m:
                return m.group(1)
        self.node.get_logger().warn(f'visual {mesh_name} not found in the Gazebo scene')
        return None

    def _spawn_patch(self, zone, index):
        p = self.panels[zone]
        self.patch_count += 1
        return self._spawn_tris(f'paint_{zone}_{self.patch_count}', p['tris'][index] + OFFSET * p['normal'][index][:, None, :])

    def _spawn_tris(self, name, tris):
        path = os.path.join(self.tmp, name + '.stl')
        g.save_stl(path, np.concatenate([tris, tris[:, [0, 2, 1]]]))
        rgba = '%.3f %.3f %.3f 1' % tuple(self.color)
        sdf = (f"<sdf version='1.9'><model name='{name}'><static>true</static><link name='link'><visual name='paint'>"
               f"<cast_shadows>false</cast_shadows><geometry><mesh><uri>file://{path}</uri></mesh></geometry>"
               f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><specular>0.25 0.25 0.3 1</specular></material>"
               f"</visual></link></model></sdf>")
        self._gz('create', 'gz.msgs.EntityFactory',
                 f'sdf: "{sdf}" pose {{position {{x: {self.car_x} y: 0 z: {self.car_z}}}}}')
        return name

    def _retire(self, names):
        # hide at once (one call), delete in the background
        if not names:
            return
        self._gz('set_pose_vector', 'gz.msgs.Pose_V', ' '.join(f'pose {{name: "{n}" position {{z: -50}}}}' for n in names))
        for n in names:
            self.remove_queue.put(n)

    def _retire_due(self):
        now = time.time()
        due = [names for t, names in self.retiring if t <= now]
        self.retiring = [(t, names) for t, names in self.retiring if t > now]
        for names in due:
            self._retire(names)

    def _remove_worker(self):
        while True:
            name = self.remove_queue.get()
            self._gz('remove', 'gz.msgs.Entity', f'name: "{name}" type: MODEL', wait=True)
            try:
                os.remove(os.path.join(self.tmp, name + '.stl'))
            except OSError:
                pass
            self.remove_queue.task_done()

    def wait_cleanup(self):
        while self.retiring:
            time.sleep(0.2)
            self._retire_due()
        self.remove_queue.join()
