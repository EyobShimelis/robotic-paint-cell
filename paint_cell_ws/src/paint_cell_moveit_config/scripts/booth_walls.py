#!/usr/bin/env python3
"""Adds the booth walls (config/booth_walls.yaml) to the MoveIt planning scene, then exits."""
import os
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


def main():
    rclpy.init()
    node = Node('booth_walls')
    path = os.path.join(get_package_share_directory('paint_cell_moveit_config'), 'config', 'booth_walls.yaml')
    walls = yaml.safe_load(open(path))['walls']

    scene = PlanningScene(is_diff=True)
    for w in walls:
        obj = CollisionObject(id=w['name'], operation=CollisionObject.ADD)
        obj.header.frame_id = 'world'
        obj.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=w['size']))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = w['position']
        pose.orientation.w = 1.0
        obj.primitive_poses.append(pose)
        scene.world.collision_objects.append(obj)

    client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
    while not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().info('waiting for move_group /apply_planning_scene ...')
    future = client.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(node, future)
    ok = future.result() is not None and future.result().success
    node.get_logger().info(f'added {len(walls)} booth walls to the planning scene: {"ok" if ok else "FAILED"}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
