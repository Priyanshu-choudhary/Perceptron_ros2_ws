"""Full ArUco auto-docking simulation: Gazebo + ros2_control + perception + RViz.

    ros2 launch perceptron_docking docking_simulation.launch.py

Docking does NOT start on its own (auto_start defaults to false) so that you can
look around, drive manually, and choose when to hand over. To dock:

    ros2 run perceptron_docking dock start        # or the raw service call:
    ros2 service call /docking/start  std_srvs/srv/Trigger {}
    ros2 service call /docking/cancel std_srvs/srv/Trigger {}

Watch progress with:
    ros2 topic echo /docking/status
    ros2 run rqt_image_view rqt_image_view /docking/debug_image

Drive it by hand with:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped

Useful arguments:
    gui:=false             headless Gazebo (much faster; RViz still works)
    rviz:=false            skip RViz
    simple_visuals:=true   primitive visuals instead of the STL meshes (WSL speed-up)
    software_gl:=true      llvmpipe fallback if WSLg cannot give Gazebo a GL context
    auto_start:=true       begin searching for the marker immediately
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_docking = FindPackageShare('perceptron_docking')
    pkg_bringup = FindPackageShare('perceptron_robot_bringup')
    pkg_gazebo = FindPackageShare('perceptron_robot_gazebo')

    args = [
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz2'),
        DeclareLaunchArgument('software_gl', default_value='false'),
        DeclareLaunchArgument('simple_visuals', default_value='false'),
        DeclareLaunchArgument(
            'auto_start',
            default_value='false',
            description='Start searching for the dock marker as soon as the node comes up',
        ),
        DeclareLaunchArgument(
            'nav2', default_value='true',
            description='Bring up Nav2 (with slam_toolbox for map -> odom). The '
                        'docking controller hands the approach leg to Nav2, so '
                        'without this there is nothing to drive it and docking '
                        'fails at APPROACH. Set false only if you are supplying '
                        'Nav2 from another launch file.'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('nav_profile', default_value='dwb'),
    ]

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'gazebo_control2.launch.py'])
        ),
        launch_arguments={
            'world_path': PathJoinSubstitution([pkg_gazebo, 'worlds', 'docking_world.world']),
            'use_sim_time': 'true',
            'gui': LaunchConfiguration('gui'),
            'software_gl': LaunchConfiguration('software_gl'),
            'simple_visuals': LaunchConfiguration('simple_visuals'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    # Perception + control. Held back until Gazebo has published a camera frame,
    # otherwise the detector logs a wall of "no image" and the controller's
    # marker watchdog fires before the first picture arrives.
    docking_pipeline = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_docking, 'launch', 'docking.launch.py'])
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'auto_start': LaunchConfiguration('auto_start'),
                    'cmd_vel_topic': '/diff_drive_controller/cmd_vel_unstamped',
                }.items(),
            )
        ],
    )

    # Nav2 drives the approach leg now, so the docking-only simulation has to
    # provide it. slam_toolbox rather than AMCL because docking_world has no
    # saved map and does not need one: the approach is a couple of metres, and
    # what Nav2 is being asked for here is obstacle avoidance on the way in, not
    # global localisation.
    nav2_stack = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('perceptron_navigation'),
                                          'launch', 'navigation.launch.py'])),
                launch_arguments={
                    'use_sim_time': 'true',
                    'slam': 'true',
                    'nav_profile': LaunchConfiguration('nav_profile'),
                }.items(),
                condition=IfCondition(LaunchConfiguration('nav2')),
            )
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_bringup, 'rviz', 'docking.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription(
        args + [gazebo_sim, nav2_stack, docking_pipeline, rviz_node])
