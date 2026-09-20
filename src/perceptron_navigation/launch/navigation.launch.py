"""Nav2 for the Perceptron base. LEVELS 4 to 7.

    level 4  NavigateToPose
    level 5  static obstacle avoidance   (global_costmap static + inflation layers)
    level 6  dynamic obstacle avoidance  (both costmaps' obstacle layer, from /scan)
    level 7  waypoint missions           (nav2 waypoint_follower, driven by mission_node)

This wraps nav2_bringup rather than re-declaring its nodes, so it keeps working
across Nav2 patch releases. Two Perceptron-specific pieces are added:

  * cmd_vel_relay, because Nav2 publishes Twist on /cmd_vel(_smoothed) while
    diff_drive_controller listens on /diff_drive_controller/cmd_vel_unstamped.
  * localisation is optional: with slam:=true, slam_toolbox supplies map -> odom
    and no AMCL or map_server runs.

Real robot:

    ros2 launch perceptron_navigation navigation.launch.py \\
        use_sim_time:=false robot_cmd_vel_topic:=/cmd_vel

With the AR path overlay on the front camera, so no separate terminal is
needed (jetson_path_overlay.py must be running on the Nano):

    ros2 launch perceptron_navigation navigation.launch.py \\
        use_sim_time:=false robot_cmd_vel_topic:=/cmd_vel overlay:=true

If localisation is already owned by localization.launch.py, add
localization:=external or two AMCLs will fight over map -> odom.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression, TextSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav2_share = FindPackageShare('nav2_bringup')
    pkg_share = FindPackageShare('perceptron_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    slam = LaunchConfiguration('slam')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='true builds the map live with slam_toolbox; '
                        'false localises against `map` with AMCL'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([pkg_share, 'maps', 'room_map.yaml']),
            description='Map yaml, used only when slam:=false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument(
            'nav_cmd_vel_topic', default_value='/cmd_vel',
            description='What Nav2 finally publishes. In Humble nav2_bringup the chain is '
                        'controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel, '
                        'because navigation_launch.py remaps cmd_vel to cmd_vel_nav on the '
                        'controller and cmd_vel_smoothed to cmd_vel on the smoother. So the '
                        'end of the chain is plain /cmd_vel, not /cmd_vel_smoothed. Verify '
                        'with: ros2 topic info /cmd_vel --verbose'),
        DeclareLaunchArgument(
            'robot_cmd_vel_topic',
            default_value='/diff_drive_controller/cmd_vel_unstamped',
            description='What the base listens on. /cmd_vel on the real robot.'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'relay', default_value='true',
            description='Run cmd_vel_relay. Needed in simulation, where the base '
                        'listens on /diff_drive_controller/cmd_vel_unstamped. On '
                        'the real robot Nav2 already ends on /cmd_vel and the '
                        'bridge subscribes there, so relaying /cmd_vel onto '
                        '/cmd_vel would be a feedback loop - pass false.'),
        # The spawn pose, so AMCL can seed itself instead of waiting for a human
        # to drag "2D Pose Estimate" in RViz before anything will move.
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'localization', default_value='auto',
            description='Who publishes map -> odom. auto follows `slam`; '
                        'slam = slam_toolbox; amcl = map_server + AMCL; '
                        'external = neither, which is what the map-frame EKF '
                        '(and VIO feeding it) needs. Exactly one publisher.'),
        DeclareLaunchArgument(
            'overlay', default_value='false',
            description='true also starts path_overlay_node, which projects '
                        '/plan into the front camera and pushes the pixels to '
                        'the Jetson to draw. Off by default because it needs '
                        'jetson_path_overlay.py running on the Nano to be of '
                        'any use, and it is a viewing aid, not part of driving.'),
        DeclareLaunchArgument(
            'overlay_jetson_ip', default_value='192.168.1.7',
            description='where path_overlay_node pushes projected pixels'),
        DeclareLaunchArgument(
            'nav_profile', default_value='dwb',
            description='dwb   = NavFn + DWB, the profile every measured number '
                        'in SIMULATION.md was taken with. '
                        'mppi  = SmacPlanner2D + MPPI, for skid-steer scrub and '
                        'graded terrain cost. Both plugin sets are always loaded; '
                        'the profile only selects which behaviour tree runs.'),
    ]

    # The profile is chosen by behaviour tree, not by a second parameter file,
    # so the costmap and server tuning below cannot drift between the two.
    bt_xml = PathJoinSubstitution([
        pkg_share, 'behavior_trees',
        [TextSubstitution(text='nav_to_pose_'),
         LaunchConfiguration('nav_profile'),
         TextSubstitution(text='.xml')]])
    profile_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': bt_xml,
            'initial_pose.x': LaunchConfiguration('initial_pose_x'),
            'initial_pose.y': LaunchConfiguration('initial_pose_y'),
            'initial_pose.yaw': LaunchConfiguration('initial_pose_yaw'),
        },
        convert_types=True)

    # LEVEL 2 path: slam_toolbox owns map -> odom.
    # `localization` wins when it is set; `auto` keeps the old slam:= behaviour
    # so every existing command line and every tools/ script still means what it
    # used to.
    localization = LaunchConfiguration('localization')
    want_slam = PythonExpression([
        "'", localization, "' == 'slam' or ('", localization,
        "' == 'auto' and '", slam, "'.lower() == 'true')"])
    want_amcl = PythonExpression([
        "'", localization, "' == 'amcl' or ('", localization,
        "' == 'auto' and '", slam, "'.lower() != 'true')"])

    # Both child launch files call their YAML argument params_file. Isolate the
    # SLAM include and supply its own YAML explicitly; otherwise it inherits
    # Nav2's params_file from this context and loses the robot's SLAM tuning.
    slam_toolbox = GroupAction(
        condition=IfCondition(want_slam),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', 'slam.launch.py'])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': PathJoinSubstitution([pkg_share, 'config', 'slam_toolbox.yaml']),
            }.items(),
        )],
    )

    # LEVEL 3 path: map_server + AMCL own map -> odom.
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_share, 'launch', 'localization_launch.py'])),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': profile_params,
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
        condition=IfCondition(want_amcl),
    )

    # LEVELS 4 to 7: planner, controller, costmaps, behaviours, BT, waypoints.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_share, 'launch', 'navigation_launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': profile_params,
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
    )

    relay = Node(
        package='perceptron_navigation',
        executable='cmd_vel_relay',
        name='cmd_vel_relay',
        output='screen',
        condition=IfCondition(LaunchConfiguration('relay')),
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': LaunchConfiguration('nav_cmd_vel_topic'),
            'output_topic': LaunchConfiguration('robot_cmd_vel_topic'),
        }],
    )

    # Viewing aid only: it subscribes to /plan and TF and publishes nothing
    # into the robot, so starting or stopping it cannot disturb navigation.
    overlay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'path_overlay.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'jetson_ip': LaunchConfiguration('overlay_jetson_ip'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('overlay')),
    )

    return LaunchDescription(
        args + [GroupAction([slam_toolbox, localization, navigation, relay,
                             overlay])])
