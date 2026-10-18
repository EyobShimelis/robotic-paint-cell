# Paint cell in Gazebo + MoveIt + RViz:  ros2 launch paint_cell_gazebo gazebo.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    desc_pkg = get_package_share_directory('paint_cell_description')
    cfg_pkg = get_package_share_directory('paint_cell_moveit_config')
    gz_pkg = get_package_share_directory('paint_cell_gazebo')
    controllers = os.path.join(cfg_pkg, 'config', 'ros2_controllers.yaml')

    moveit_config = (
        MoveItConfigsBuilder('paint_cell', package_name='paint_cell_moveit_config')
        .robot_description(file_path=os.path.join(desc_pkg, 'urdf', 'paint_cell.urdf.xacro'),
                           mappings={'hardware': 'gazebo', 'controllers_file': controllers,
                                     'car_start': LaunchConfiguration('car_start')})
        .robot_description_semantic(file_path='config/paint_cell.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(pipelines=['ompl', 'pilz_industrial_motion_planner'], default_planning_pipeline='ompl')
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .to_moveit_configs()
    )
    sim_time = {'use_sim_time': True}
    use_rviz = LaunchConfiguration('use_rviz')

    # lets Gazebo resolve package:// mesh paths
    resource_paths = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(get_package_share_directory(p)))
        for p in ('paint_cell_description', 'kuka_quantec_support')
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v2 ', LaunchConfiguration('world'),
                                      PythonExpression(["'' if '", LaunchConfiguration('gz_gui'),
                                                        "' == 'true' else ' -s --headless-rendering'"])],
                          'on_exit_shutdown': 'true'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[moveit_config.robot_description, sim_time])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'paint_cell'])

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    spawners = [
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=[name, '--controller-manager', '/controller_manager', '--controller-manager-timeout', '60'])
        for name in ('joint_state_broadcaster', 'arm_controller', 'track_controller', 'conveyor_controller')
    ]
    start_controllers = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawners))

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[moveit_config.to_dict(), sim_time, {'publish_robot_description_semantic': True}])

    booth_walls = Node(package='paint_cell_moveit_config', executable='booth_walls.py', output='screen', parameters=[sim_time])

    rviz = Node(
        package='rviz2', executable='rviz2', output='log', condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(cfg_pkg, 'rviz', 'moveit.rviz')],
        parameters=[moveit_config.robot_description, moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics, moveit_config.planning_pipelines,
                    moveit_config.joint_limits, sim_time])

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        # 0 = car at the paint position, -10 = waiting outside the entry door
        DeclareLaunchArgument('car_start', default_value='0.0'),
        DeclareLaunchArgument('gz_gui', default_value='true'),   # false: no Gazebo window
        DeclareLaunchArgument('world', default_value=os.path.join(gz_pkg, 'worlds', 'paint_booth.sdf')),
        *resource_paths, gazebo, robot_state_publisher, spawn, clock_bridge, start_controllers,
        move_group, booth_walls, rviz,
    ])
