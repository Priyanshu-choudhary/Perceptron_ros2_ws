from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  # <-- ADD THIS IMPORT
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Use substitution-safe methods to find packages and paths
    description_pkg = FindPackageShare('perceptron_robot_description')
    control_pkg = FindPackageShare('perceptron_robot_control')
    gazebo_ros_pkg = FindPackageShare('gazebo_ros')

    # Use PathJoinSubstitution for robust path building
    urdf_file = PathJoinSubstitution([description_pkg, 'urdf', 'perceptron_robot.xacro'])
    controllers_yaml = PathJoinSubstitution([control_pkg, 'config', 'controllers.yaml'])
    # For world file, we can use the direct function since it's used in a non-substitution context
    world_file = os.path.join(get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world')

    gui_arg = DeclareLaunchArgument('gui', default_value='true', description='Set to "false" to run headless.')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use sim clock')

    # Produce robot_description. This is a Substitution that will be evaluated later.
    robot_description_content = Command(
        [FindExecutable(name='xacro'), ' ', urdf_file]
    )

    # FIX: Create a ParameterValue that will be evaluated to a string
    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_param,  # <-- USE ParameterValue here
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Start gazebo - Use the direct function here since IncludeLaunchDescription doesn't use substitutions
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file, 'verbose': 'false', 'gui': LaunchConfiguration('gui')}.items()
    )

    # Spawn robot (publishes robot_description on topic)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'perceptron_robot', '-z', '0.1'],
        output='screen'
    )

    # The controller_manager node - ALSO NEEDS THE FIX
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_param},  # <-- USE ParameterValue here too
            controllers_yaml
        ],
        output='screen',
    )

    # Spawners — they talk to the controller_manager node we just started
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Strategy: 
    # 1. Start gazebo and the control_node (which contains the controller_manager)
    # 2. Spawn the entity
    # 3. After spawn, start the controllers

    start_joint_state_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    start_diff_after_joint = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner]
        )
    )

    return LaunchDescription([
        gui_arg,
        use_sim_time_arg,
        gazebo_launch,
        control_node,
        robot_state_pub_node,
        spawn_entity,
        start_joint_state_after_spawn,
        start_diff_after_joint
    ])