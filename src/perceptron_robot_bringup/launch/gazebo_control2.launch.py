"""Bring the Perceptron robot up in Gazebo Classic with ros2_control.

Layout of the stack this launches:

    gzserver ──> libgazebo_ros2_control.so  ──> controller_manager (lives INSIDE gzserver)
                                                  ├── joint_state_broadcaster
                                                  └── diff_drive_controller
    robot_state_publisher ──> /tf, /robot_description
    spawn_entity.py       ──> pulls /robot_description into Gazebo

Note there is deliberately no separate `ros2_control_node` here: with
gazebo_ros2_control the controller manager is hosted by the Gazebo plugin.
Starting a second one is a common cause of "controller manager not available"
and of controllers that claim interfaces nobody is writing.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'world_path',
            default_value=PathJoinSubstitution(
                [FindPackageShare('perceptron_robot_gazebo'), 'worlds', 'docking_world.world']
            ),
            description='World file to load',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true', description='Run gzclient'),
        DeclareLaunchArgument(
            'software_gl',
            default_value='false',
            description=(
                'Force the llvmpipe software rasteriser. Only needed if WSLg cannot give '
                'Gazebo a GL context; it is roughly 10x slower, so leave it off unless '
                'gzclient crashes or renders black.'
            ),
        ),
        DeclareLaunchArgument(
            'simple_visuals',
            default_value='false',
            description=(
                'Replace the ~340k-triangle STL visuals with boxes/cylinders. '
                'Big frame-rate win on WSL; the physics is identical either way.'
            ),
        ),
        DeclareLaunchArgument(
            'wheel_collision', default_value='cylinder',
            description='cylinder on flat ground; sphere on heightmap terrain, '
                        'where ODE can hang on cylinder-heightfield contacts.'),
        DeclareLaunchArgument(
            'camera_type', default_value='mono',
            description='mono, stereo (left/right pair) or depth (RGBD cloud). '
                        '/camera/image_raw is published in all three so ArUco '
                        'docking keeps working.'),
        DeclareLaunchArgument('stereo_baseline', default_value='0.12'),
        DeclareLaunchArgument(
            'lidar_type', default_value='2d',
            description='2d (LD19, /scan), 3d (16-beam tilted, /points), or both'),
        DeclareLaunchArgument(
            'lidar_pitch', default_value='0.26',
            description='Downward tilt of the 3D scanner in radians. 3d/both only.'),
        DeclareLaunchArgument(
            'use_ekf', default_value='true',
            description=(
                'Fuse wheel odometry with the IMU through robot_localization. '
                'This node publishes odom -> base_footprint; with it off, nothing '
                'does, because diff_drive_controller has enable_odom_tf: false.'
            ),
        ),
        DeclareLaunchArgument(
            'global_ekf', default_value='false',
            description='Also run a second robot_localization filter in the map '
                        'frame, which then owns map -> odom. Requires '
                        'localization:=external so nothing else publishes it.'),
        DeclareLaunchArgument(
            'simulate_discharge', default_value='false',
            description='Drain the simulated pack over time so battery low and '
                        'critical actually fire during a run.'),
        DeclareLaunchArgument('discharge_minutes_to_empty', default_value='30.0'),
        DeclareLaunchArgument(
            'use_battery', default_value='true',
            description='Publish sensor_msgs/BatteryState on /battery/state. '
                        'Simulated values by default; change them live with '
                        'ros2 param set /battery_node voltage <V>.'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        # base_footprint sits on the ground, so a 2 cm drop is plenty. Spawning
        # higher makes the robot bounce and can fling a wheel joint.
        DeclareLaunchArgument('z', default_value='0.02'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---------------- environment ----------------
    software_gl = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value='1',
        condition=IfCondition(LaunchConfiguration('software_gl')),
    )

    # Gazebo needs to find both our dock model and the robot meshes referenced
    # as package://perceptron_robot_description/meshes/...  Adding the parent of
    # the package share directory makes the package:// URI resolvable.
    gz_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            PathJoinSubstitution([FindPackageShare('perceptron_robot_gazebo'), 'models']),
            ':',
            # small_house.world pulls in 89 model:// furniture references that
            # all live here. Without this the world loads as a bare floor and
            # nothing explains why.
            PathJoinSubstitution(
                [FindPackageShare('aws_robomaker_small_house_world'), 'models']),
            ':',
            # Gazebo's own stock models. Normally Gazebo finds these itself, but
            # this launch both overrides GAZEBO_MODEL_PATH and blanks
            # GAZEBO_MODEL_DATABASE_URI, and between them ground_plane and sun
            # stop resolving: "Unable to find uri[model://ground_plane]", after
            # which the world loads as a void and the lidar returns nothing.
            # room_world.world never showed this because gen_world.py writes
            # explicit SDF geometry rather than model:// includes - the stock
            # turtlebot3 worlds do use them.
            '/usr/share/gazebo-11/models',
            ':',
            PathJoinSubstitution([FindPackageShare('perceptron_robot_description'), '..']),
            ':',
            EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''),
        ],
    )
    gz_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=[
            PathJoinSubstitution([FindPackageShare('perceptron_robot_description'), '..']),
            ':',
            EnvironmentVariable('GAZEBO_RESOURCE_PATH', default_value='/usr/share/gazebo-11'),
        ],
    )
    # Empty database URI => never reach out to the online model DB, which
    # otherwise blocks startup for ~30 s on a fresh machine.
    gz_no_online_db = SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value='')

    # ---------------- robot description ----------------
    # Every path is quoted. Command() shlex-splits the string it assembles, so an
    # unquoted workspace path containing a space turns into several arguments and
    # xacro fails with "expected exactly one input file as argument".
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'),
            ' "',
            PathJoinSubstitution(
                [FindPackageShare('perceptron_robot_description'), 'urdf', 'perceptron_robot.xacro']
            ),
            '"',
            ' is_sim:=true',
            ' simple_visuals:=', LaunchConfiguration('simple_visuals'),
            ' wheel_collision:=', LaunchConfiguration('wheel_collision'),
            ' camera_type:=', LaunchConfiguration('camera_type'),
            ' stereo_baseline:=', LaunchConfiguration('stereo_baseline'),
            ' lidar_type:=', LaunchConfiguration('lidar_type'),
            ' lidar_pitch:=', LaunchConfiguration('lidar_pitch'),
            ' gazebo_controllers:="',
            PathJoinSubstitution(
                [FindPackageShare('perceptron_robot_control'), 'config', 'controllers.yaml']
            ),
            '"',
        ]
    )
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # ---------------- gazebo ----------------
    gzserver = ExecuteProcess(
        cmd=[
            'gzserver',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            LaunchConfiguration('world_path'),
        ],
        output='screen',
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_perceptron',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'perceptron_robot',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
            # spawn_entity.py defaults to giving up after 30 s. Added to the
            # 4 s timer below that is ~34 s to have /spawn_entity advertised,
            # which a small generated world always manages and a furnished one
            # with hundreds of textured meshes does not - especially on a cold
            # cache. Miss it and spawn_entity exits with "Service /spawn_entity
            # unavailable. Was Gazebo started with GazeboRosFactory?", which
            # reads like a plugin problem and is really just a slow load. The
            # robot then never appears, so there is no odom frame, no robot
            # model in RViz, and nothing for SLAM to map.
            '-timeout', '300',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # Give gzserver a moment to advertise /spawn_entity before asking for it.
    # The timer only avoids a pointless first attempt; the -timeout above is
    # what actually covers a slow world load.
    delayed_spawn = TimerAction(period=4.0, actions=[spawn_entity])

    # Sensor fusion. Wheel odometry says how far, the gyro says which way; on a
    # skid-steer the wheels are systematically wrong about yaw because turning
    # means scrubbing every tyre. See config/ekf.yaml for what is fused and,
    # more importantly, what is deliberately not.
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('perceptron_robot_bringup'), 'config', 'ekf.yaml']),
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration('use_ekf')),
    )

    # The map-frame filter. Off by default because AMCL and slam_toolbox already
    # publish map -> odom and only one thing may. Turn it on together with
    # localization:=external, which stops both of those from running.
    ekf_global = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('perceptron_robot_bringup'), 'config',
                 'ekf_global.yaml']),
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(LaunchConfiguration('global_ekf')),
    )

    # Battery monitor. Lives in perceptron_hardware because on the real robot
    # it reads the STM32 telemetry; here it publishes simulated values so the
    # docking and recharge logic has something to decide on.
    battery = Node(
        package='perceptron_hardware',
        executable='battery_node',
        name='battery_node',
        output='screen',
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('perceptron_hardware'), 'config', 'battery_params.yaml']),
            {'use_sim_time': use_sim_time,
             # Exposed so a patrol can actually be driven to its low and
             # critical thresholds in a few minutes instead of never.
             'simulate_discharge': ParameterValue(
                 LaunchConfiguration('simulate_discharge'), value_type=bool),
             'discharge_minutes_to_empty': ParameterValue(
                 LaunchConfiguration('discharge_minutes_to_empty'), value_type=float)},
        ],
        condition=IfCondition(LaunchConfiguration('use_battery')),
    )

    # ---------------- controllers ----------------
    # The controller manager only exists once the model (and therefore the
    # gazebo_ros2_control plugin) has been spawned, hence the chain below.
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager', '--controller-manager-timeout', '60'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    spawn_diff_drive = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '-c', '/controller_manager', '--controller-manager-timeout', '60'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity, on_exit=[spawn_joint_state_broadcaster]
        )
    )
    after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster, on_exit=[spawn_diff_drive]
        )
    )

    ld = LaunchDescription(args)
    for action in (
        software_gl,
        gz_model_path,
        gz_resource_path,
        gz_no_online_db,
        node_robot_state_publisher,
        gzserver,
        gzclient,
        delayed_spawn,
        ekf,
        ekf_global,
        battery,
        after_spawn,
        after_jsb,
    ):
        ld.add_action(action)
    return ld
