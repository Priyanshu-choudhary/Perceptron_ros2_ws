"""AR overlay of the Nav2 plan on the robot's front camera.

    ros2 launch perceptron_navigation path_overlay.launch.py

This is the HOST half only. It projects /plan into camera pixels and pushes
them to the Jetson over ZMQ; it never touches an image. The Jetson half is
jetson/jetson_path_overlay.py in this repo, which must be copied to the Jetson
and started there:

    # on the Jetson, camera released by --no-aruco
    python3 jetson_robot_bridge.py --no-aruco
    python3 jetson_path_overlay.py --host-ip <this machine>

    # on the operator machine
    gst-launch-1.0 -v udpsrc port=5000 \
      caps="application/x-rtp, media=video, encoding-name=H265, payload=96" \
      ! rtph265depay ! h265parse ! avdec_h265 ! autovideosink sync=false

Run this ALONGSIDE navigation.launch.py, which owns Nav2, AMCL and the map.
It needs /plan and a TF chain reaching camera_optical_link, and contributes
nothing to either, so starting or stopping it cannot disturb navigation.

Arguments:
    params       override the parameter file
    jetson_ip    default 192.168.1.7, as everywhere else
    use_sim_time set true to drive the overlay from Gazebo
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('perceptron_navigation')
    default_params = os.path.join(pkg, 'config', 'path_overlay.yaml')

    params = LaunchConfiguration('params')
    jetson_ip = LaunchConfiguration('jetson_ip')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('params', default_value=default_params),
        DeclareLaunchArgument('jetson_ip', default_value='192.168.1.7'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='perceptron_navigation',
            executable='path_overlay_node',
            name='path_overlay_node',
            output='screen',
            parameters=[
                params,
                {'jetson_ip': jetson_ip, 'use_sim_time': use_sim_time},
            ],
        ),
    ])
