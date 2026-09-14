"""Everything at once: Gazebo room + robot + Nav2 (+ SLAM) + RViz.

Build a map (level 2):

    ros2 launch perceptron_navigation nav_simulation.launch.py slam:=true
    # drive around with teleop, then:
    ros2 run nav2_map_server map_saver_cli -f <workspace>/src/perceptron_navigation/maps/room_map

Navigate on the saved map (levels 3 to 6):

    ros2 launch perceptron_navigation nav_simulation.launch.py slam:=false
    # then set the initial pose in RViz ("2D Pose Estimate") and give it a
    # goal ("2D Goal Pose")

Run the patrol mission (levels 7 and 10):

    ros2 run perceptron_navigation mission start
"""

import subprocess

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _preflight(context):
    """Refuse to launch on top of a Gazebo that is already running.

    Only one gzserver can hold port 11345. A second one exits 255, the robot is
    never spawned, and what you actually see is every Nav2 server complaining
    that frame "odom" does not exist, an empty RobotModel in RViz and no map -
    none of which mentions Gazebo. Failing here with the real reason costs one
    line and saves that whole hunt.
    """
    try:
        running = subprocess.run(['pgrep', '-x', 'gzserver'],
                                 capture_output=True, text=True)
    except (OSError, ValueError):
        return []                      # no pgrep: skip the check, do not block
    if running.returncode == 0 and running.stdout.strip():
        pids = ' '.join(running.stdout.split())
        raise RuntimeError(
            f'gzserver is already running (pid {pids}). Only one can hold port '
            '11345, so this launch would come up without a robot. Stop it '
            'first with:  pkill -9 -f gzserver')
    return []


def _validate_exploration(context):
    def value(name):
        return LaunchConfiguration(name).perform(context).lower()
    if value('exploration') != 'true':
        return []
    localization = value('localization')
    if not (localization == 'slam' or (localization == 'auto' and value('slam') == 'true')):
        raise RuntimeError('exploration:=true requires slam:=true with localization:=auto '
                           'or localization:=slam')
    # .get with a default, not value(): patrol is optional, and resolving an
    # unset LaunchConfiguration raises rather than returning empty.
    if (context.launch_configurations.get('patrol', 'false').lower() == 'true'
            and value('exploration') != 'true'):
        raise RuntimeError('patrol:=true needs exploration:=true')
    if value('lidar_type') not in ('2d', 'both'):
        raise RuntimeError('Exploration currently requires /scan: use lidar_type:=2d or both')
    if value('global_ekf') == 'true':
        raise RuntimeError('SLAM exploration owns map -> odom; set global_ekf:=false')
    if value('nav_profile') not in ('dwb', 'mppi'):
        raise RuntimeError('nav_profile must be dwb or mppi')
    return []


def generate_launch_description():
    pkg_nav = FindPackageShare('perceptron_navigation')
    pkg_bringup = FindPackageShare('perceptron_robot_bringup')
    pkg_gazebo = FindPackageShare('perceptron_robot_gazebo')

    args = [
        DeclareLaunchArgument('slam', default_value='true',
                              description='true maps as it goes, false localises on `map`'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution(
                [pkg_nav, 'maps', 'small_house_map.yaml']),
            description='Used only when localising with AMCL. Must match the '
                        'world: room_map.yaml goes with room_world.world.'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('simple_visuals', default_value='true',
                              description='Nav2 plus Gazebo is heavy; primitives by default'),
        DeclareLaunchArgument('software_gl', default_value='false'),
        # Spawn pose for small_house.world: upstream route waypoint 1, measured
        # clear by 0.636 m against a 0.33 m robot radius.
        #
        # yaw stays 0. It was briefly 1.33 (the heading that route waypoint
        # carries), which quietly broke every other world: override x and y for
        # room_world and you still inherited a 76 degree spawn rotation, SLAM
        # anchored the map frame to it, and every exploration goal came out
        # rotated 76 degrees from world coordinates. Clearance at this spot is a
        # property of the position, not the heading, so nothing is lost.
        #
        # For room_world.world use x:=-3.0 y:=-2.0, which is what waypoints.yaml
        # and the saved room_map are written against.
        DeclareLaunchArgument('x', default_value='5.18'),
        DeclareLaunchArgument('y', default_value='1.50'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('docking', default_value='true',
                              description='Also run the ArUco docking pipeline'),
        DeclareLaunchArgument('mission', default_value='true',
                              description='Also run mission_node, which serves '
                                          '/mission/start and /mission/cancel'),
        DeclareLaunchArgument('exploration', default_value='false',
                              description='Exclusive exploration/search mode; suppresses patrol '
                                          'and docking nodes. Start motion with explore start/search.'),
        DeclareLaunchArgument('search_marker_id', default_value='-1'),
        DeclareLaunchArgument('marker_dictionary', default_value='DICT_5X5_250'),
        DeclareLaunchArgument('marker_size', default_value='0.15'),
        DeclareLaunchArgument('exploration_auto_start', default_value='false'),
        DeclareLaunchArgument(
            'patrol', default_value='false',
            description='Run the patrol supervisor on top of exploration: find '
                        'the dock, save it, drive to it, dock, and return home '
                        'or stop on battery low/critical. Needs exploration:=true.'),
        DeclareLaunchArgument('dock_store_path',
                              default_value='~/.ros/perceptron_dock/dock.json'),
        DeclareLaunchArgument('simulate_discharge', default_value='false'),
        DeclareLaunchArgument('discharge_minutes_to_empty', default_value='30.0'),
        DeclareLaunchArgument('exploration_output', default_value='~/.ros/perceptron_exploration'),
        DeclareLaunchArgument('exploration_config', default_value=PathJoinSubstitution(
            [pkg_nav, 'config', 'exploration.yaml'])),
        # The world. room_world is the indoor stack everything was tuned on;
        # lunar_arena.world is the competition terrain, which needs the 3D lidar
        # and the traversability layer to be navigable at all.
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution(
                [pkg_gazebo, 'worlds', 'small_house.world'])),
        DeclareLaunchArgument('lidar_type', default_value='2d',
                              description='2d, 3d or both'),
        DeclareLaunchArgument('nav_profile', default_value='dwb',
                              description='dwb = NavFn + DWB (the measured '
                                          'baseline), mppi = SmacPlanner2D + MPPI'),
        DeclareLaunchArgument(
            'localization', default_value='auto',
            description='Who publishes map -> odom. auto follows `slam`; '
                        'slam = slam_toolbox; amcl = map_server + AMCL; '
                        'external = neither, for the global EKF / VIO path. '
                        'Exactly one thing may publish that transform.'),
        DeclareLaunchArgument(
            'vio', default_value='false',
            description='Run visual odometry into /vio/odom for the map-frame '
                        'EKF to fuse. Needs camera_type:=depth.'),
        DeclareLaunchArgument(
            'vio_strategy', default_value='feature',
            description='feature = visual features, icp = geometric depth '
                        'registration. See launch/vio.launch.py.'),
        DeclareLaunchArgument(
            'global_ekf', default_value='false',
            description='Run the second robot_localization filter in the map '
                        'frame. Use with localization:=external.'),
        DeclareLaunchArgument('camera_type', default_value='mono',
                              description='mono, stereo or depth'),
        DeclareLaunchArgument('stereo_baseline', default_value='0.12'),
        # Terrain analysis. Off indoors, where a flat floor makes it pure cost:
        # decoding a point cloud at 10 Hz is the most expensive thing in the
        # stack on this machine, and a room has nothing for it to find.
        DeclareLaunchArgument(
            'traversability', default_value='false',
            description='Run traversability_node, which turns /points into the '
                        'lethal cells that feed both costmaps. Needs '
                        'lidar_type:=3d or both.'),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_bringup, 'launch', 'gazebo_control2.launch.py'])),
        launch_arguments={
            'world_path': LaunchConfiguration('world'),
            'use_sim_time': 'true',
            'lidar_type': LaunchConfiguration('lidar_type'),
            'camera_type': LaunchConfiguration('camera_type'),
            'stereo_baseline': LaunchConfiguration('stereo_baseline'),
            'gui': LaunchConfiguration('gui'),
            'software_gl': LaunchConfiguration('software_gl'),
            'simple_visuals': LaunchConfiguration('simple_visuals'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
            'global_ekf': LaunchConfiguration('global_ekf'),
            'simulate_discharge': LaunchConfiguration('simulate_discharge'),
            'discharge_minutes_to_empty':
                LaunchConfiguration('discharge_minutes_to_empty'),
        }.items(),
    )

    # Wait for the robot to actually exist before starting Nav2, rather than
    # guessing with a timer.
    #
    # This is not cosmetic. Nav2 brought up before the robot exists has no
    # odom -> base_link transform, so local_costmap spins on
    #     Invalid frame ID "odom" ... frame does not exist
    # controller_server blocks part-way through its lifecycle activation,
    # bt_navigator never activates, and goals are rejected with no obvious
    # explanation. A fixed timer long enough on one machine is short on the
    # next.
    #
    # The gate is the odom -> base_link transform itself, not a sensor topic.
    # Waiting on /scan looked equivalent and was not: /scan appears the moment
    # Gazebo finishes spawning the model, whereas odom -> base_link only
    # appears once spawn_entity has exited, joint_state_broadcaster has been
    # spawned, and diff_drive_controller after it. Measured cold start of
    # small_house on this machine: /scan at 40.2 s, odom -> base_link at
    # 42.7 s. The old barrier released Nav2 three seconds after /scan, so it
    # was racing the controllers with about half a second in hand - a race
    # headless runs happened to win and runs with the Gazebo GUI and RViz
    # competing for the machine lost.
    #
    # Watching the transform also drops the lidar_type special case: there is
    # no /scan at all with lidar_type:=3d, so the old gate had to know which
    # topic to watch. Odometry does not care what is fitted.
    wait_for_robot = ExecuteProcess(
        cmd=['ros2', 'run', 'perceptron_navigation', 'wait_for_odom_tf'],
        output='screen',
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav, 'launch', 'navigation.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'slam': LaunchConfiguration('slam'),
            'map': LaunchConfiguration('map'),
            'nav_profile': LaunchConfiguration('nav_profile'),
            'localization': LaunchConfiguration('localization'),
            # AMCL starts believing it is where Gazebo put the robot.
            'initial_pose_x': LaunchConfiguration('x'),
            'initial_pose_y': LaunchConfiguration('y'),
            'initial_pose_yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    docking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('perceptron_docking'),
                                  'launch', 'docking.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'auto_start': 'false',
            'cmd_vel_topic': '/diff_drive_controller/cmd_vel_unstamped',
        }.items(),
        # Exploration suppresses docking - except under patrol, which exists to
        # hand the robot over to the docking controller once it has found the
        # marker. Without this exception the supervisor reaches the staging pose
        # and then waits forever on a /docking/start that nobody is serving.
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('docking'), "'.lower() == 'true' and ('",
            LaunchConfiguration('exploration'), "'.lower() != 'true' or '",
            LaunchConfiguration('patrol'), "'.lower() == 'true')"])),
    )

    # The patrol state machine. It only serves its services; nothing happens
    # until /mission/start is called.
    mission = Node(
        package='perceptron_navigation',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Which localiser to wait on before the first goal.
            'slam': ParameterValue(LaunchConfiguration('slam'), value_type=bool),
        }],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('mission'), "'.lower() == 'true' and '",
            LaunchConfiguration('exploration'), "'.lower() != 'true'"])),
    )

    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [pkg_nav, 'launch', 'exploration.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            **{name: LaunchConfiguration(name) for name in (
                'nav_profile', 'search_marker_id', 'marker_dictionary', 'marker_size',
                'exploration_auto_start', 'exploration_output', 'exploration_config',
                'patrol', 'dock_store_path')},
        }.items(),
        condition=IfCondition(LaunchConfiguration('exploration')),
    )

    # Visual odometry, if asked for. Started with the rest of the stack because
    # it needs camera frames and the robot's transforms to exist first.
    vio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav, 'launch', 'vio.launch.py'])),
        launch_arguments={
            'use_sim_time': 'true',
            'vio_strategy': LaunchConfiguration('vio_strategy'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('vio')),
    )

    # Started with the rest of the stack rather than up front, because it needs
    # the robot's transforms to exist before its first cloud arrives.
    # Where the cloud comes from depends on what is fitted. The 3D lidar puts
    # one on /points; the depth camera puts one on /depth_camera/points. The
    # classifier does not care which - it bins whatever arrives into an
    # elevation grid - so this is a topic choice, not a second node.
    #
    # They are not equivalent in coverage, though. The lidar sweeps 360 deg;
    # the depth camera sees about 60 deg ahead out to a few metres, so a single
    # frame is far less situational awareness. The rolling local costmap is what
    # gives the robot memory of what it has already driven past.
    cloud_topic = PythonExpression([
        "'/depth_camera/points' if '", LaunchConfiguration('camera_type'),
        "' == 'depth' else '/points'"])
    traversability = Node(
        package='perceptron_navigation',
        executable='traversability_node',
        name='traversability_node',
        output='screen',
        parameters=[{'use_sim_time': True, 'input_topic': cloud_topic}],
        condition=IfCondition(LaunchConfiguration('traversability')),
    )

    start_stack = RegisterEventHandler(
        OnProcessExit(target_action=wait_for_robot,
                      on_exit=[navigation, docking, mission, traversability, vio, exploration]))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_nav, 'rviz', 'navigation.rviz'])],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription(args + [OpaqueFunction(function=_preflight),
                                     OpaqueFunction(function=_validate_exploration),
                                     gazebo, wait_for_robot, start_stack, rviz])
