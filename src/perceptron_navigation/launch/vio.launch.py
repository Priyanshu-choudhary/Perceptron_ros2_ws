"""Visual odometry from the depth camera, for the map-frame EKF to fuse.

    ros2 launch perceptron_navigation vio.launch.py

WHY VIO AT ALL
Wheel odometry and the IMU drift without bound, and the two things that normally
correct that - AMCL against a saved map, slam_toolbox matching live scans - both
need geometry within laser range to match against. Put the robot in the middle
of an open arena with no wall inside 12 m and there is nothing for either of
them to bite on. A camera still sees the ground it is driving over.

WHY rtabmap's rgbd_odometry
It ships as a Humble binary (`ros-humble-rtabmap-odom`), it publishes a plain
nav_msgs/Odometry with a real covariance, and robot_localization can therefore
consume it as just another source. That last point is the whole design: VIO does
not replace the EKF, it becomes an input to it, so losing vision degrades the
estimate instead of ending it.

WHAT IT IS NOT
This publishes no TF. `odom -> base_footprint` belongs to the odom-frame EKF and
`map -> odom` to the map-frame one; a third publisher would fight them. The only
output is the topic.

TEXTURE, AND THE HONEST CAVEAT
`vio_strategy:=feature` (default) tracks visual features. That is the stronger
choice indoors and anywhere with texture. It is the WEAKER choice on lunar
regolith, which is close to uniform in albedo - the same objection the URDF
already raises about stereo matching on sand.

`vio_strategy:=icp` registers the depth cloud geometrically instead. Fine sand
has no texture but a cratered surface has plenty of SHAPE, and that is what ICP
uses. Expect feature to win in the room and ICP to win in the arena; measure it
with tools/localizationtest.sh rather than assuming.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _is_icp():
    return PythonExpression(
        ["'", LaunchConfiguration('vio_strategy'), "' == 'icp'"])


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'vio_strategy', default_value='feature',
            description='feature = visual features (needs texture); '
                        'icp = geometric registration of the depth cloud '
                        '(needs relief, not texture)'),
        # camera_type:=depth publishes the RGB stream on /camera/image_raw and
        # the depth image alongside it, already registered - same sensor, same
        # intrinsics, same optical frame - which is exactly what rgbd_odometry
        # needs and what it cannot check for you.
        DeclareLaunchArgument('rgb_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info'),
        DeclareLaunchArgument('depth_topic',
                              default_value='/depth_camera/depth/image_raw'),
        DeclareLaunchArgument('odom_topic', default_value='/vio/odom'),
        # icp_odometry works from the cloud, not the images.
        DeclareLaunchArgument('cloud_topic',
                              default_value='/depth_camera/points'),
    ]

    remaps = [
        ('rgb/image', LaunchConfiguration('rgb_topic')),
        ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
        ('depth/image', LaunchConfiguration('depth_topic')),
        ('odom', LaunchConfiguration('odom_topic')),
    ]

    common = {
        'use_sim_time': use_sim_time,
        'frame_id': 'base_footprint',
        # A private odom frame id that nothing subscribes to. publish_tf is off,
        # so this only ever labels the message header.
        'odom_frame_id': 'vio_odom',
        'publish_tf': False,
        # Gazebo stamps the RGB and depth images from the same render but they
        # do not arrive with identical stamps; exact sync drops nearly every
        # pair.
        'approx_sync': True,
        'queue_size': 30,
        # Publish a zero-motion estimate rather than nothing when tracking is
        # lost, so the EKF sees a covariance blow-up instead of silence.
        'publish_null_when_lost': False,
    }

    feature_odom = Node(
        package='rtabmap_odom', executable='rgbd_odometry', name='vio_odometry',
        output='screen', remappings=remaps,
        parameters=[dict(common, **{
            'Odom/Strategy': '0',        # 0 = Frame-to-Map
            'Vis/CorType': '0',          # features matched by descriptor
            'Vis/MinInliers': '15',
        })],
        condition=UnlessCondition(_is_icp()),
    )

    # Geometric odometry is a DIFFERENT NODE, not rgbd_odometry with an ICP flag
    # set. rgbd_odometry is a visual pipeline all the way down: point it at an
    # untextured scene and it reports "Not enough inliers 0/15" every frame
    # whatever the registration strategy says, because it never gets as far as
    # registration. icp_odometry consumes the depth cloud directly and never
    # looks at appearance at all.
    icp_odom = Node(
        package='rtabmap_odom', executable='icp_odometry', name='vio_odometry',
        output='screen',
        remappings=[('scan_cloud', LaunchConfiguration('cloud_topic')),
                    ('odom', LaunchConfiguration('odom_topic'))],
        parameters=[dict(common, **{
            'expected_update_rate': 0.0,
            'deskewing': False,
            'Icp/VoxelSize': '0.05',
            'Icp/MaxCorrespondenceDistance': '0.20',
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '10',
            'Icp/Epsilon': '0.001',
            # A depth camera sees a cone, not a sweep, so it is far easier for
            # one frame to be a single dominant plane - a wall, or the floor.
            # Plane-to-plane ICP cannot recover translation along that plane, so
            # keep frames until there is real shape change to register against.
            'Odom/ScanKeyFrameThr': '0.8',
            'OdomF2M/ScanMaxSize': '15000',
        })],
        condition=IfCondition(_is_icp()),
    )

    return LaunchDescription(args + [feature_odom, icp_odom])
