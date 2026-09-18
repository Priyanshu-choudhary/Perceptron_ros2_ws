#!/usr/bin/env python3
"""Absolute pose from wall-mounted ArUco boards, used to seed AMCL.

    Jetson  detectMarkers -> corner pixels  --ZMQ b"aruco"-->  this node
    here    solvePnP -> T_base_board -> T_map_base -> /initialpose

WHY THIS SEEDS AMCL INSTEAD OF PUBLISHING map -> odom ITSELF

`localization.launch.py` and `ekf_global.yaml` both spell out the rule: exactly
one node may publish map -> odom, because tf2 resolves two publishers by
interleaving them and the robot appears to teleport several times a second.
AMCL already owns that transform and is better than this node everywhere
except at startup, where it has no idea where it is at all. So this node does
the one thing AMCL cannot do -- produce an absolute fix from nothing -- hands
it over on /initialpose, and then gets out of the way.

That also means `set_initial_pose` must be FALSE in nav2_params.yaml on the
real robot. Left true, AMCL starts confidently at the origin and burns its
particle budget before the first board is ever seen.

WHY THE CORNERS ARRIVE OVER ZMQ RATHER THAN AS AN IMAGE TOPIC

Detection needs raw sensor pixels and a lossy stream destroys exactly the
corner precision the pose depends on, so it runs on the Jetson. See
ArucoThread in jetson_robot_bridge.py. Everything downstream of detection is
in marker_map.py, with no ROS imports, so a simulation path that starts from
/camera/image_raw can reuse it unchanged.

TIMESTAMPING

Like jetson_bridge_node, observations are stamped with the host clock on
receipt rather than the Jetson's, so the two machines' clocks need not agree.
The cost is an unmodelled pipeline latency of roughly 50-150 ms. That is
harmless here only because a fix is refused while the robot is moving faster
than `max_seed_speed` -- a stationary robot's pose does not go stale.
"""

import math
import os
import threading
from collections import deque

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_srvs.srv import Trigger

import msgpack
import zmq

from perceptron_navigation import marker_map as mm


def quaternion_to_rotation(x, y, z, w):
    """3x3 rotation from a quaternion. Normalised first; TF is not exact."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError('zero-length quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def transform_to_matrix(transform_stamped):
    """geometry_msgs/TransformStamped -> 4x4."""
    t = transform_stamped.transform.translation
    q = transform_stamped.transform.rotation
    return mm.make_transform(quaternion_to_rotation(q.x, q.y, q.z, q.w),
                             (t.x, t.y, t.z))


class ArucoLocalizerNode(Node):

    def __init__(self):
        super().__init__('aruco_localizer_node')

        default_jetson_ip = os.environ.get('JETSON_IP', '192.168.1.7')
        self.declare_parameter('jetson', '')
        self.declare_parameter('jetson_ip', default_jetson_ip)
        self.declare_parameter('telemetry_port', 5555)
        self.declare_parameter('marker_map_path', '')
        self.declare_parameter('dictionary_name', 'DICT_6X6_250')

        # Intrinsics. Defaults are the measured values from camera_calib.json
        # at the resolution they were measured at; the node rescales them if
        # the Jetson ever streams a different size.
        self.declare_parameter('camera_matrix', [
            410.9212916385953, 0.0, 634.8042096493966,
            0.0, 411.4395116912116, 347.58359871233904,
            0.0, 0.0, 1.0])
        self.declare_parameter('dist_coeffs', [
            0.030287865182229395, -0.002860707267771121,
            -0.004223044189411946, -0.0003374289867825264,
            -0.0003820853552867496])
        self.declare_parameter('calibration_width', 1280)
        self.declare_parameter('calibration_height', 720)

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_optical_frame', 'camera_optical_link')

        # --- gating. Every one of these exists because passing a bad fix to
        # --- AMCL is worse than passing none: the filter converges onto it.
        self.declare_parameter('max_reprojection_error', 2.0)
        self.declare_parameter('min_tiles', 2)
        self.declare_parameter('max_range', 2.0)
        self.declare_parameter('min_range', 0.25)
        self.declare_parameter('edge_margin_fraction', 0.15)
        self.declare_parameter('agreement_frames', 5)
        self.declare_parameter('agreement_position_tolerance', 0.05)
        self.declare_parameter('agreement_yaw_tolerance', 0.05)
        self.declare_parameter('max_seed_speed', 0.02)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('require_odom', True)

        # --- output
        self.declare_parameter('auto_seed', True)
        self.declare_parameter('reseed_interval', 0.0)
        self.declare_parameter('seed_position_stddev', 0.05)
        self.declare_parameter('seed_yaw_stddev', 0.03)

        get = self.get_parameter
        self.map_frame = get('map_frame').value
        self.base_frame = get('base_frame').value
        self.camera_optical_frame = get('camera_optical_frame').value
        self.expected_dict = get('dictionary_name').value

        self.max_reprojection_error = float(get('max_reprojection_error').value)
        self.min_tiles = int(get('min_tiles').value)
        self.max_range = float(get('max_range').value)
        self.min_range = float(get('min_range').value)
        self.edge_margin = float(get('edge_margin_fraction').value)
        self.agreement_frames = max(2, int(get('agreement_frames').value))
        self.agreement_position = float(get('agreement_position_tolerance').value)
        self.agreement_yaw = float(get('agreement_yaw_tolerance').value)
        self.max_seed_speed = float(get('max_seed_speed').value)
        self.require_odom = bool(get('require_odom').value)

        self.auto_seed = bool(get('auto_seed').value)
        self.reseed_interval = float(get('reseed_interval').value)
        self.position_stddev = float(get('seed_position_stddev').value)
        self.yaw_stddev = float(get('seed_yaw_stddev').value)

        self.calibrated_size = (int(get('calibration_width').value),
                                int(get('calibration_height').value))
        self.base_camera_matrix = np.array(
            get('camera_matrix').value, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(get('dist_coeffs').value, dtype=np.float64)

        self.marker_map = None
        self.inert_reason = None
        path = get('marker_map_path').value
        if not path:
            self._go_inert('marker_map_path is empty -- nothing to localise against')
        else:
            try:
                self.marker_map = mm.load_marker_map(path)
                self.get_logger().info(
                    'Loaded %d board(s) from %s (map_reference=%s)'
                    % (len(self.marker_map['boards']), path,
                       self.marker_map.get('map_reference', 'UNSET')))
                if not self.marker_map.get('map_reference'):
                    self.get_logger().warn(
                        'marker map has no map_reference. A board pose in the '
                        '"map" frame only means something against the map it '
                        'was surveyed on -- record which one.')
            except Exception as exc:
                self._go_inert('cannot load marker map %s: %s' % (path, exc))

        self.window = deque(maxlen=self.agreement_frames)
        self.inbox = deque(maxlen=20)
        self.inbox_lock = threading.Lock()
        self.seed_armed = self.auto_seed
        self.last_seed_time = None
        self.last_speed = None
        self.last_speed_time = None
        self.frames_seen = 0
        self.frames_rejected = 0
        self.last_reject_reason = ''

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Transient-local so RViz and anything started later still sees the
        # last fix rather than waiting for the robot to look at a board again.
        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/aruco/map_pose', latched)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.create_subscription(Odometry, get('odom_topic').value,
                                 self._odom_cb, 10)
        self.create_service(Trigger, '/aruco/relocalize', self._relocalize_cb)

        self.running = True
        self.zmq_thread = None
        if self.marker_map is not None:
            j_val = str(get('jetson').value or '').strip()
            j_ip_val = str(get('jetson_ip').value or '').strip()
            target_ip = j_val if j_val else (j_ip_val or default_jetson_ip)
            self.jetson_url = 'tcp://%s:%d' % (target_ip,
                                               int(get('telemetry_port').value))
            self.zmq_thread = threading.Thread(target=self._zmq_worker, daemon=True)
            self.zmq_thread.start()

        self.create_timer(0.05, self._process_inbox)
        self.create_timer(10.0, self._report_status)

    # ------------------------------------------------------------- lifecycle

    def _go_inert(self, reason):
        """Stay up, do nothing, and say so. Repeatedly.

        Refusing to construct would take the whole launch down with it, and
        this node is an addition to a robot that navigates fine without it.
        """
        self.inert_reason = reason
        self.get_logger().error('ArUco localiser INERT: %s' % reason)

    def destroy_node(self):
        self.running = False
        super().destroy_node()

    # ----------------------------------------------------------------- input

    def _odom_cb(self, msg):
        linear = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.last_speed = max(linear, abs(msg.twist.twist.angular.z))
        self.last_speed_time = self.get_clock().now()

    def _zmq_worker(self):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, b'aruco')
        socket.setsockopt(zmq.RCVTIMEO, 1000)
        # Only the newest observation matters. Without a shallow watermark a
        # stall builds a backlog and the node then works through a queue of
        # poses describing where the robot used to be.
        socket.set_hwm(5)
        socket.connect(self.jetson_url)
        self.get_logger().info('Subscribed to ArUco corners at %s' % self.jetson_url)

        while self.running and rclpy.ok():
            try:
                frames = socket.recv_multipart()
            except zmq.Again:
                continue
            except Exception as exc:
                if self.running:
                    self.get_logger().warn('ArUco ZMQ receive failed: %s' % exc)
                continue
            # b"aruco_debug" carries JPEG bytes on the same prefix subscription.
            if len(frames) < 2 or frames[0] != b'aruco':
                continue
            try:
                payload = msgpack.unpackb(frames[1], raw=False)
            except Exception:
                continue
            with self.inbox_lock:
                self.inbox.append(payload)

        socket.close()
        context.term()

    # ------------------------------------------------------------ processing

    def _process_inbox(self):
        with self.inbox_lock:
            if not self.inbox:
                return
            # Drop everything but the newest: older observations are strictly
            # worse evidence and processing them only adds latency.
            payload = self.inbox[-1]
            self.inbox.clear()
        self._handle_observation(payload)

    def _reject(self, reason):
        self.frames_rejected += 1
        self.last_reject_reason = reason
        self.window.clear()

    def _handle_observation(self, payload):
        if self.marker_map is None:
            return
        self.frames_seen += 1

        if payload.get('dict') != self.expected_dict:
            self._reject('dictionary mismatch: Jetson sends %s, node expects %s'
                         % (payload.get('dict'), self.expected_dict))
            self.get_logger().error(self.last_reject_reason, throttle_duration_sec=10.0)
            return

        ids = payload.get('ids') or []
        if not ids:
            self.window.clear()
            return

        name, board, known_ids = mm.board_for_ids(self.marker_map, ids)
        if board is None:
            self._reject('detected ids %s are not in the marker map' % ids)
            return

        corners_by_id = dict(zip(ids, payload.get('corners') or []))
        object_points, image_points, used = mm.assemble_correspondences(
            board, known_ids, [corners_by_id[i] for i in known_ids])
        if len(used) < self.min_tiles:
            self._reject('only %d tile(s) of board "%s", need %d'
                         % (len(used), name, self.min_tiles))
            return

        width = int(payload.get('width', self.calibrated_size[0]))
        height = int(payload.get('height', self.calibrated_size[1]))
        if not mm.corners_are_central(image_points, width, height, self.edge_margin):
            self._reject('board "%s" is too close to the frame edge, where the '
                         'wide-angle distortion model is least trustworthy' % name)
            return

        camera_matrix = mm.scale_intrinsics(
            self.base_camera_matrix, self.calibrated_size, (width, height))
        camera_from_board, error = mm.solve_board_pose(
            object_points, image_points, camera_matrix, self.dist_coeffs)
        if camera_from_board is None:
            self._reject('PnP failed for board "%s"' % name)
            return
        if error > self.max_reprojection_error:
            self._reject('reprojection error %.2f px > %.2f for board "%s"'
                         % (error, self.max_reprojection_error, name))
            return

        distance = float(np.linalg.norm(camera_from_board[:3, 3]))
        if distance > self.max_range or distance < self.min_range:
            self._reject('board "%s" at %.2f m is outside [%.2f, %.2f]'
                         % (name, distance, self.min_range, self.max_range))
            return

        try:
            base_from_camera = transform_to_matrix(self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_optical_frame, rclpy.time.Time()))
        except Exception as exc:
            self._reject('no TF %s -> %s: %s'
                         % (self.base_frame, self.camera_optical_frame, exc))
            self.get_logger().warn(self.last_reject_reason, throttle_duration_sec=10.0)
            return

        base_from_board = base_from_camera.dot(camera_from_board)
        pose = mm.robot_pose_in_map(mm.map_from_board(board), base_from_board)
        self.window.append(pose)

        if len(self.window) < self.agreement_frames:
            return
        if not mm.poses_agree(self.window, self.agreement_position,
                              self.agreement_yaw):
            self._reject('the last %d fixes on board "%s" disagree by more than '
                         '%.3f m / %.3f rad' % (self.agreement_frames, name,
                                                self.agreement_position,
                                                self.agreement_yaw))
            return

        x, y, yaw = mm.mean_pose(self.window)
        self._publish(x, y, yaw, name, len(used), error, distance)

    # ---------------------------------------------------------------- output

    def _publish(self, x, y, yaw, board_name, tiles, error, distance):
        msg = self._pose_msg(x, y, yaw)
        self.pose_pub.publish(msg)

        if not self._may_seed():
            return

        self.initialpose_pub.publish(msg)
        self.last_seed_time = self.get_clock().now()
        self.window.clear()
        if self.reseed_interval <= 0.0:
            self.seed_armed = False
        self.get_logger().info(
            'SEEDED /initialpose from board "%s": x=%.3f y=%.3f yaw=%.1f deg '
            '(%d tiles, %.2f m, reproj %.2f px)%s'
            % (board_name, x, y, math.degrees(yaw), tiles, distance, error,
               '' if self.reseed_interval > 0.0
               else '. One-shot: call /aruco/relocalize to seed again.'))

    def _may_seed(self):
        if not self.seed_armed:
            if self.reseed_interval <= 0.0:
                return False
            if self.last_seed_time is None:
                return False
            age = (self.get_clock().now() - self.last_seed_time).nanoseconds * 1e-9
            if age < self.reseed_interval:
                return False

        if self.last_speed is None:
            if self.require_odom:
                self.get_logger().warn(
                    'Holding a good fix back: no odometry yet, so "is the robot '
                    'stationary?" cannot be answered. Set require_odom:=false to '
                    'seed anyway.', throttle_duration_sec=10.0)
                return False
        elif self.last_speed > self.max_seed_speed:
            self.get_logger().info(
                'Holding a good fix back: robot is moving at %.3f (limit %.3f). '
                'The vision pipeline latency is unmodelled, so a fix taken while '
                'moving would be stale by an unknown amount.'
                % (self.last_speed, self.max_seed_speed),
                throttle_duration_sec=5.0)
            return False
        return True

    def _pose_msg(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = mm.yaw_to_quaternion(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        covariance = [0.0] * 36
        covariance[0] = self.position_stddev ** 2
        covariance[7] = self.position_stddev ** 2
        covariance[35] = self.yaw_stddev ** 2
        msg.pose.covariance = covariance
        return msg

    def _relocalize_cb(self, request, response):
        if self.marker_map is None:
            response.success = False
            response.message = 'inert: %s' % self.inert_reason
            return response
        self.seed_armed = True
        self.window.clear()
        self.last_seed_time = None
        response.success = True
        response.message = ('Armed. Point the camera at a mapped board and hold '
                            'still; the next %d agreeing fixes will seed AMCL.'
                            % self.agreement_frames)
        self.get_logger().info(response.message)
        return response

    def _report_status(self):
        if self.inert_reason is not None:
            self.get_logger().error('ArUco localiser INERT: %s' % self.inert_reason)
            return
        self.get_logger().info(
            'ArUco: %d frames, %d rejected, seed %s%s'
            % (self.frames_seen, self.frames_rejected,
               'ARMED' if self.seed_armed else 'done',
               (' | last reject: ' + self.last_reject_reason)
               if self.last_reject_reason else ''))


def main(args=None):
    rclpy.init(args=args)
    node = ArucoLocalizerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
