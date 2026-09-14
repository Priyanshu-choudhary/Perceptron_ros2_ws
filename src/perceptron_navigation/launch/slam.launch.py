"""slam_toolbox in online asynchronous mode. LEVEL 2, build the map.

Standalone so it works on the real robot too:

    ros2 launch perceptron_navigation slam.launch.py use_sim_time:=false

Drive around, watch the map fill in in RViz, then save it:

    ros2 run nav2_map_server map_saver_cli -f src/perceptron_navigation/maps/room_map

That writes room_map.pgm and room_map.yaml. Rebuild afterwards so the map is
installed, or point the `map` argument at the source path directly.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('perceptron_navigation'), 'config', 'slam_toolbox.yaml']),
            description='slam_toolbox parameter file',
        ),
    ]

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(args + [slam])
