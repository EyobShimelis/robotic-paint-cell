# MoveIt + RViz with mock hardware (no Gazebo):  ros2 launch paint_cell_moveit_config demo.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    desc_pkg = get_package_share_directory('paint_cell_description')
    cfg_pkg = get_package_share_directory('paint_cell_moveit_config')

    moveit_config = (
        MoveItConfigsBuilder('paint_cell', package_name='paint_cell_moveit_config')
        .robot_description(file_path=os.path.join(desc_pkg, 'urdf', 'paint_cell.urdf.xacro'),
                           mappings={'hardware': 'mock', 'car_start': LaunchConfiguration('car_start')})
        .robot_description_semantic(file_path='config/paint_cell.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl', 'pilz_industrial_motion_planner'], default_planning_pipeline='ompl')
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .to_moveit_configs()
    )

    use_rviz = LaunchConfiguration('use_rviz')

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[moveit_config.robot_description])

    ros2_control_node = Node(
        package='controller_manager', executable='ros2_control_node', output='screen',
        parameters=[os.path.join(cfg_pkg, 'config', 'ros2_controllers.yaml')],
        remappings=[('~/robot_description', '/robot_description')])

    spawners = [
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=[name, '--controller-manager', '/controller_manager'])
        for name in ('joint_state_broadcaster', 'arm_controller', 'track_controller', 'conveyor_controller')
    ]

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[moveit_config.to_dict(), {'publish_robot_description_semantic': True}])

    booth_walls = Node(package='paint_cell_moveit_config', executable='booth_walls.py', output='screen')

    rviz = Node(
        package='rviz2', executable='rviz2', output='log', condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(cfg_pkg, 'rviz', 'moveit.rviz')],
        parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics, moveit_config.planning_pipelines,
                    moveit_config.joint_limits])

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        # 0 = car at the paint position, -10 = waiting outside the entry door
        DeclareLaunchArgument('car_start', default_value='0.0'),
        robot_state_publisher, ros2_control_node, *spawners, move_group, booth_walls, rviz,
    ])
