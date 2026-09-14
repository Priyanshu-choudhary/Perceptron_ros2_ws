"""Synthetic isolated ROS graph: real detector/coordinator, fake Nav2 and sensors.

No simulator, wheel-command publisher, or real robot connection is used.
"""

import json
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster

from perceptron_navigation.aruco_search_detector import ArucoSearchDetector, dictionary_for
from perceptron_navigation.exploration_node import ExplorationNode


class SyntheticWorld(Node):
    def __init__(self):
        super().__init__('synthetic_exploration_world')
        cb = ReentrantCallbackGroup()
        self.bridge = CvBridge()
        self.x, self.y = 1.5, 3.0
        self.nav_goals, self.spins, self.cancelled = [], 0, False
        self.relay_enabled = False
        self.scan_enabled = True
        self.fully_mapped = False
        self.visible_id = 42  # should not satisfy a request for ID 10
        self.broadcast = TransformBroadcaster(self)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map',
                                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 5)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 5)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 5)
        self.create_service(SetBool, '/cmd_vel_relay/enable', self._relay, callback_group=cb)
        for name in ('bt_navigator', 'controller_server', 'behavior_server'):
            self.create_service(GetState, '/' + name + '/get_state', self._lifecycle,
                                callback_group=cb)
        self.nav_action = ActionServer(self, NavigateToPose, '/navigate_to_pose', self._navigate,
                                       cancel_callback=lambda _: CancelResponse.ACCEPT,
                                       callback_group=cb)
        self.spin_action = ActionServer(self, Spin, '/spin', self._spin,
                                        cancel_callback=lambda _: CancelResponse.ACCEPT,
                                        callback_group=cb)
        self.create_timer(0.1, self._sensors, callback_group=cb)

    def _relay(self, request, response):
        self.relay_enabled = request.data
        response.success = True
        return response

    def _lifecycle(self, request, response):
        response.current_state.id = 3
        response.current_state.label = 'active'
        return response

    def _spin(self, handle):
        self.spins += 1
        time.sleep(0.1)
        if handle.is_cancel_requested:
            handle.canceled()
        else:
            handle.succeed()
        return Spin.Result()

    def _navigate(self, handle):
        self.nav_goals.append(handle.request)
        self.x = handle.request.pose.pose.position.x
        self.y = handle.request.pose.pose.position.y
        self.visible_id = 10  # the marker is revealed after leaving the start
        deadline = time.monotonic() + 10
        while rclpy.ok() and time.monotonic() < deadline:
            if handle.is_cancel_requested:
                self.cancelled = True
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.02)
        handle.abort()
        return NavigateToPose.Result()

    def _sensors(self):
        stamp = self.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x, tf.transform.translation.y = self.x, self.y
        tf.transform.rotation.w = 1.0
        optical = TransformStamped()
        optical.header.stamp = stamp
        optical.header.frame_id = 'base_footprint'
        optical.child_frame_id = 'camera_optical_link'
        optical.transform.translation.x = 0.18
        optical.transform.translation.z = 0.16
        q = optical.transform.rotation
        q.x, q.y, q.z, q.w = -0.5, 0.5, -0.5, 0.5
        self.broadcast.sendTransform([tf, optical])

        grid = np.full((60, 80), 100, dtype=np.int8)
        grid[10:50, 5:35] = 0
        grid[10:50, 35:75] = -1
        if self.nav_goals:
            grid[10:50, 35:55] = 0
        if self.fully_mapped:
            grid[10:50, 5:75] = 0
        msg = OccupancyGrid()
        msg.header.stamp, msg.header.frame_id = stamp, 'map'
        msg.info.width, msg.info.height, msg.info.resolution = 80, 60, 0.1
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.ravel().tolist()
        self.map_pub.publish(msg)
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = stamp, 'laser_link'
        if self.scan_enabled:
            self.scan_pub.publish(scan)

        image = np.full((600, 800), 255, dtype=np.uint8)
        image[220:340, 340:460] = cv2.aruco.drawMarker(
            dictionary_for('DICT_5X5_250'), self.visible_id, 120)
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding='mono8')
        image_msg.header.stamp, image_msg.header.frame_id = stamp, 'camera_optical_link'
        info = CameraInfo()
        info.header = image_msg.header
        info.width, info.height = 800, 600
        info.k = [476.7, 0., 400., 0., 476.7, 300., 0., 0., 1.]
        info.d = [0.] * 5
        self.info_pub.publish(info)
        self.image_pub.publish(image_msg)


def test_real_ros_pipeline_searches_then_confirms_and_cancels(tmp_path):
    rclpy.init(domain_id=187, signal_handler_options=SignalHandlerOptions.NO)
    coordinator = ExplorationNode(parameter_overrides=[
        Parameter('target_marker_id', value=10),
        Parameter('output_directory', value=str(tmp_path)),
        Parameter('startup_timeout', value=20.0),
    ])
    detector = ArucoSearchDetector()
    world = SyntheticWorld()
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (coordinator, detector, world):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        # Start through the actual service, not by manually setting node state.
        client = world.create_client(Trigger, '/exploration/start')
        assert client.wait_for_service(timeout_sec=5)
        response = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            if coordinator.result_file is not None:
                break
            time.sleep(0.05)
        assert response.done() and response.result().success
        assert coordinator.state == 'FOUND', coordinator.detail
        assert world.relay_enabled
        assert world.spins == 4
        assert len(world.nav_goals) == 1
        assert world.cancelled, 'FOUND must wait for the active Nav2 goal to stop'
        assert coordinator.motion is None
        assert coordinator.found['id'] == 10
        assert coordinator.found['position']['x'] > world.x + 0.5
        result = json.loads(Path(coordinator.result_file).read_text())
        assert result['state'] == 'FOUND'
        assert Path(result['map_yaml']).exists()
        assert world.nav_goals[0].behavior_tree.endswith('explore_dwb.xml')
    finally:
        executor.shutdown(timeout_sec=12)
        thread.join(timeout=2)
        world.nav_action.destroy()
        world.spin_action.destroy()
        for node in (coordinator, detector, world):
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('scenario', ['map_complete', 'user_cancel', 'scan_failure'])
def test_terminal_conditions_stop_and_save(tmp_path, scenario):
    rclpy.init(domain_id=187, signal_handler_options=SignalHandlerOptions.NO)
    coordinator = ExplorationNode(parameter_overrides=[
        Parameter('target_marker_id', value=-1),
        Parameter('output_directory', value=str(tmp_path)),
        Parameter('startup_timeout', value=20.0),
        Parameter('sensor_timeout', value=1.0),
    ])
    world = SyntheticWorld()
    world.fully_mapped = scenario == 'map_complete'
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(coordinator)
    executor.add_node(world)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        start = world.create_client(Trigger, '/exploration/start')
        cancel = world.create_client(Trigger, '/exploration/cancel')
        assert start.wait_for_service(timeout_sec=5)
        start.call_async(Trigger.Request())
        triggered = False
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if world.nav_goals and not triggered:
                if scenario == 'user_cancel':
                    cancel.call_async(Trigger.Request())
                elif scenario == 'scan_failure':
                    world.scan_enabled = False
                triggered = True
            if coordinator.result_file is not None:
                break
            time.sleep(0.05)
        expected = {'map_complete': 'COMPLETE', 'user_cancel': 'CANCELLED',
                    'scan_failure': 'FAILED'}[scenario]
        assert coordinator.state == expected, coordinator.detail
        assert coordinator.motion is None
        if scenario != 'map_complete':
            assert world.cancelled
        if scenario == 'scan_failure':
            assert 'stale' in coordinator.detail
        assert json.loads(Path(coordinator.result_file).read_text())['state'] == expected
    finally:
        executor.shutdown(timeout_sec=12)
        thread.join(timeout=2)
        world.nav_action.destroy()
        world.spin_action.destroy()
        coordinator.destroy_node()
        world.destroy_node()
        rclpy.shutdown()
