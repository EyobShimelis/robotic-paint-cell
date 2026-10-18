"""Thin wrappers around the move_group services and actions used by the paint planner."""
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotState, RobotTrajectory
from moveit_msgs.srv import GetPositionFK, GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
TRACK = ['track_x', 'track_y']
CONVEYOR = ['conveyor_x']


def duration(seconds):
    sec = int(seconds)
    return Duration(sec=sec, nanosec=int(round((seconds - sec) * 1e9)))


class MoveItIO:
    def __init__(self, node):
        self.node = node
        self.joints = {}
        self.velocities = {}
        self.transit_scale = 1.0          # moves between passes, fraction of the joint limits
        self.last_planner = ''
        node.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self.ik = node.create_client(GetPositionIK, '/compute_ik')
        self.fk = node.create_client(GetPositionFK, '/compute_fk')
        self.validity = node.create_client(GetStateValidity, '/check_state_validity')
        self.move_group = ActionClient(node, MoveGroup, '/move_action')
        self.execute = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')

    def _on_joint_states(self, msg):
        self.joints.update(zip(msg.name, msg.position))
        if len(msg.velocity) == len(msg.name):
            self.velocities.update(zip(msg.name, msg.velocity))

    def wait_ready(self, timeout=60.0):
        for client in (self.ik, self.fk, self.validity):
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f'move_group service {client.srv_name} not available')
        for action in (self.move_group, self.execute):
            if not action.wait_for_server(timeout_sec=timeout):
                raise RuntimeError('move_group action server not available')
        for controller in ('arm_controller', 'track_controller', 'conveyor_controller'):
            client = ActionClient(self.node, FollowJointTrajectory, f'/{controller}/follow_joint_trajectory')
            if not client.wait_for_server(timeout_sec=timeout):
                raise RuntimeError(f'{controller} is not running')
            client.destroy()
        end = time.time() + timeout
        while len(self.joints) < 9 and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if len(self.joints) < 9:
            raise RuntimeError('no /joint_states received')

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    def current(self, names):
        self.spin(0.05)
        return [self.joints[n] for n in names]

    @staticmethod
    def state(values):
        """dict joint -> position  ->  RobotState"""
        s = RobotState()
        s.joint_state = JointState(name=list(values), position=[float(v) for v in values.values()])
        s.is_diff = True
        return s

    def solve_ik(self, position, quaternion, state, avoid_collisions=True, timeout=0.05):
        """IK for paint_tcp at a world pose, seeded with `state` (dict of all joints). Returns arm joints or None."""
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name, r.ik_link_name, r.avoid_collisions = 'arm', 'paint_tcp', avoid_collisions
        r.robot_state = self.state(state)
        r.pose_stamped = PoseStamped()
        r.pose_stamped.header.frame_id = 'world'
        p, o = r.pose_stamped.pose.position, r.pose_stamped.pose.orientation
        p.x, p.y, p.z = (float(v) for v in position)
        o.x, o.y, o.z, o.w = (float(v) for v in quaternion)
        r.timeout = duration(timeout)
        res = self._call(self.ik, req)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def tcp_pose(self, state, link='paint_tcp'):
        req = GetPositionFK.Request()
        req.header.frame_id = 'world'
        req.fk_link_names = [link]
        req.robot_state = self.state(state)
        return self._call(self.fk, req).pose_stamped[0].pose

    def is_valid(self, state):
        """Collision check of a full robot state. Returns (valid, list of colliding link pairs)."""
        req = GetStateValidity.Request()
        req.robot_state = self.state(state)
        res = self._call(self.validity, req)
        return res.valid, sorted({tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in res.contacts})

    def _plan_and_execute(self, group, goal, pipeline, planner, velocity, acceleration, planning_time, attempts, on_spin=None):
        g = MoveGroup.Goal()
        req = g.request
        req.group_name, req.pipeline_id, req.planner_id = group, pipeline, planner
        req.num_planning_attempts, req.allowed_planning_time = attempts, planning_time
        req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor = velocity, acceleration
        c = Constraints()
        c.joint_constraints = [JointConstraint(joint_name=k, position=float(v), tolerance_above=1e-3, tolerance_below=1e-3, weight=1.0)
                               for k, v in goal.items()]
        req.goal_constraints = [c]
        handle_future = self.move_group.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, handle_future)
        result_future = handle_future.result().get_result_async()
        while not result_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if on_spin:
                on_spin()
        self.spin(0.2)
        return result_future.result().result.error_code.val == MoveItErrorCodes.SUCCESS

    def move_joints(self, group, goal, velocity=None, acceleration=None, planning_time=5.0, on_spin=None, straight_only=False):
        """Straight joint move (Pilz PTP) first; if blocked, OMPL (retried, it is random). True on success."""
        velocity = self.transit_scale if velocity is None else velocity
        acceleration = self.transit_scale if acceleration is None else acceleration
        self.last_planner = 'PTP'
        if self._plan_and_execute(group, goal, 'pilz_industrial_motion_planner', 'PTP', velocity, acceleration, planning_time, 1, on_spin):
            return True
        if straight_only:
            return False
        self.last_planner = 'OMPL'
        # half speed: OMPL paths can run close to the joint limits
        velocity, acceleration = min(velocity, 0.5), min(acceleration, 0.5)
        for _ in range(3):
            if self._plan_and_execute(group, goal, 'ompl', '', velocity, acceleration, planning_time, 5, on_spin):
                return True
        return False

    def execute_trajectory(self, names, points, times, velocities=None):
        """Execute a timed joint trajectory (MoveIt splits it between the arm and track controllers)."""
        traj = JointTrajectory(joint_names=list(names))
        for i, (q, t) in enumerate(zip(points, times)):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q], time_from_start=duration(t))
            if velocities is not None:
                pt.velocities = [float(v) for v in velocities[i]]
            traj.points.append(pt)
        goal = ExecuteTrajectory.Goal(trajectory=RobotTrajectory(joint_trajectory=traj))
        handle_future = self.execute.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, handle_future)
        self.result_future = handle_future.result().get_result_async()
        return self.result_future

    def wait_result(self, future, on_spin=None):
        while not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if on_spin:
                on_spin()
        self.spin(0.2)
        return future.result().result.error_code.val == MoveItErrorCodes.SUCCESS
