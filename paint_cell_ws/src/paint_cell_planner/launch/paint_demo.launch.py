# Full demo: car enters, robot paints, car leaves.  ros2 launch paint_cell_planner paint_demo.launch.py
# zones:="[roof,hood]" paints only some zones (default: all)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('paint_cell_gazebo'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'car_start': LaunchConfiguration('car_start'), 'use_rviz': LaunchConfiguration('use_rviz'),
                          'gz_gui': LaunchConfiguration('gz_gui'), 'world': LaunchConfiguration('world')}.items())

    paint_job = Node(
        package='paint_cell_planner', executable='paint_job', output='screen',
        parameters=[{'use_sim_time': True, 'gazebo': True,
                     'zones': ParameterValue(LaunchConfiguration('zones'), value_type=None),
                     'output_dir': LaunchConfiguration('output_dir')}])

    # used to move the paint with the car on the conveyor
    set_pose_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/world/paint_booth/set_pose@ros_gz_interfaces/srv/SetEntityPose'])

    return LaunchDescription([
        DeclareLaunchArgument('car_start', default_value='-10.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('gz_gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=os.path.join(get_package_share_directory('paint_cell_gazebo'), 'worlds', 'paint_booth.sdf')),
        DeclareLaunchArgument('zones', default_value='[all]'),
        DeclareLaunchArgument('output_dir', default_value='paint_results'),
        gazebo, set_pose_bridge, paint_job,
    ])
