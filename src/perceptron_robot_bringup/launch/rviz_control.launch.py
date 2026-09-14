from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_desc = get_package_share_directory('perceptron_robot_description')
    pkg_ctrl = get_package_share_directory('perceptron_robot_control')

    xacro_file = PathJoinSubstitution([pkg_desc, 'urdf', 'perceptron_robot.xacro'])
    controllers_yaml = PathJoinSubstitution([pkg_ctrl, 'config', 'controllersRviz.yaml'])

    # Quoted: Command() shlex-splits, so a workspace path with a space would
    # otherwise reach xacro as several arguments.
    robot_description = ParameterValue(Command([
        'xacro "', xacro_file, '"',
        ' is_sim:=false',
    ]), value_type=str)


    return LaunchDescription([
        # RobotStatePublisher with our xacro
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        # Controller Manager (ros2_control_node)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description}, controllers_yaml],
            output='screen',
        ),

        # Spawners (start broadcaster first, then controllers)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen',
        ),
       

        # RViz (optional; you can reuse your display.launch.py if preferred)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])
