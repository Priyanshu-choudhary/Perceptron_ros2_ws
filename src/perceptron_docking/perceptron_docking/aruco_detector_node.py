#!/usr/bin/env python3
"""ArUco marker detection and pose estimation for the Perceptron docking stack.

Subscribes : /camera/image_raw, /camera/camera_info
Publishes  : /docking/marker_pose      (PoseStamped, in base_frame)
             /docking/marker_detected  (Bool, every frame)
             /docking/debug_image      (Image, annotated)
             TF  camera_optical_frame -> aruco_dock_marker_<id>

The pose is published in base_frame (base_footprint) so the controller can work
in plain robot coordinates: +x forward, +y left. If that transform is not
available the pose is NOT published - an optical-frame pose looks like a valid
message but has x=right / z=forward, which would send the robot sideways.
"""

import math

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped with tf2)
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

DICT_NAMES = (
    'DICT_4X4_50', 'DICT_4X4_100', 'DICT_4X4_250', 'DICT_4X4_1000',
    'DICT_5X5_50', 'DICT_5X5_100', 'DICT_5X5_250', 'DICT_5X5_1000',
    'DICT_6X6_50', 'DICT_6X6_100', 'DICT_6X6_250', 'DICT_6X6_1000',
    'DICT_7X7_50', 'DICT_7X7_100', 'DICT_7X7_250', 'DICT_7X7_1000',
    'DICT_ARUCO_ORIGINAL',
)


def rotation_matrix_to_quaternion(R: np.ndarray) -> tuple:
    """3x3 rotation matrix -> normalised quaternion (x, y, z, w)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s

    n = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / n, y / n, z / n, w / n) if n > 0.0 else (0.0, 0.0, 0.0, 1.0)


class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('aruco_detector_node')

        self.declare_parameter('marker_id', 42)
        self.declare_parameter('marker_size', 0.10)
        self.declare_parameter('dictionary_name', 'DICT_5X5_250')
        self.declare_parameter('camera_image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('use_camera_info', True)
        self.declare_parameter('camera_optical_frame', 'camera_optical_link')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('marker_frame_prefix', 'aruco_dock_marker')
        self.declare_parameter('fallback_camera_matrix',
                               [554.38, 0.0, 320.0, 0.0, 554.38, 240.0, 0.0, 0.0, 1.0])
        self.declare_parameter('fallback_dist_coeffs', [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_topic', '/docking/debug_image')
        # Detection is expensive and only useful near the dock. Something that
        # knows when docking is about to start - mission_node, say - can switch
        # it off for the rest of the time through /docking/detector/enable.
        # Default on, so standalone docking runs need no extra step.
        self.declare_parameter('enabled', True)

        p = self.get_parameter
        self.target_marker_id = int(p('marker_id').value)
        self.marker_size = float(p('marker_size').value)
        self.dict_name = p('dictionary_name').value
        self.camera_optical_frame = p('camera_optical_frame').value
        self.base_frame = p('base_frame').value
        self.marker_frame_prefix = p('marker_frame_prefix').value
        self.use_camera_info = bool(p('use_camera_info').value)
        self.publish_debug = bool(p('publish_debug_image').value)

        self.aruco_dict = self._get_aruco_dict(self.dict_name)
        self.aruco_params = (aruco.DetectorParameters_create()
                             if hasattr(aruco, 'DetectorParameters_create')
                             else aruco.DetectorParameters())
        # Sub-pixel corner refinement roughly halves the pose jitter, which is
        # the difference between a +/-2 cm dock and a wobbling one.
        self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        # Built once. The old code constructed a detector per frame.
        self.detector = (aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
                         if hasattr(aruco, 'ArucoDetector') else None)

        self.camera_matrix = np.array(p('fallback_camera_matrix').value,
                                      dtype=np.float64).reshape((3, 3))
        self.dist_coeffs = np.array(p('fallback_dist_coeffs').value, dtype=np.float64)
        self.camera_info_received = not self.use_camera_info

        # Marker corner model, matching OpenCV's corner order (TL, TR, BR, BL).
        # This puts marker +z out of the printed face, towards the camera.
        s = self.marker_size / 2.0
        self.marker_3d_corners = np.array(
            [[-s, s, 0.0], [s, s, 0.0], [s, -s, 0.0], [-s, -s, 0.0]], dtype=np.float32)

        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker_pose_pub = self.create_publisher(PoseStamped, '/docking/marker_pose', 10)
        self.detected_pub = self.create_publisher(Bool, '/docking/marker_detected', 10)
        if self.publish_debug:
            self.debug_image_pub = self.create_publisher(Image, p('debug_image_topic').value, 10)

        sensor_qos = QoSProfile(depth=5,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        # BEST_EFFORT matches both best-effort and reliable publishers; a
        # RELIABLE subscriber would silently never connect to a best-effort one.
        self.create_subscription(Image, p('camera_image_topic').value, self._image_cb, sensor_qos)
        if self.use_camera_info:
            self.create_subscription(CameraInfo, p('camera_info_topic').value,
                                     self._camera_info_cb, 10)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.create_service(SetBool, '/docking/detector/enable', self._srv_enable)

        self._frames_seen = 0
        self.get_logger().info(
            f'ArUco detector up: id={self.target_marker_id}, '
            f'{self.marker_size * 100:.1f} cm, dict={self.dict_name}, '
            f'reporting poses in {self.base_frame}')

    def _get_aruco_dict(self, name: str):
        if name not in DICT_NAMES or not hasattr(aruco, name):
            self.get_logger().warn(f'Unknown dictionary "{name}", falling back to DICT_5X5_250')
            name = 'DICT_5X5_250'
        return aruco.getPredefinedDictionary(getattr(aruco, name))

    def _camera_info_cb(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.camera_info_received = True
            self.get_logger().info(
                f'Camera intrinsics received: fx={self.camera_matrix[0, 0]:.1f} '
                f'cx={self.camera_matrix[0, 2]:.1f} cy={self.camera_matrix[1, 2]:.1f}')

    def _detect(self, gray):
        if self.detector is not None:
            return self.detector.detectMarkers(gray)
        return aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

    def _srv_enable(self, request, response):
        self.enabled = bool(request.data)
        response.success = True
        response.message = f'detector {"enabled" if self.enabled else "disabled"}'
        self.get_logger().info(response.message)
        return response

    def _image_cb(self, msg: Image):
        if not self.enabled:
            # Return before any OpenCV work. Publishing "not detected" keeps
            # consumers' watchdogs honest rather than letting them read a stale
            # true from before the detector was switched off.
            self.detected_pub.publish(Bool(data=False))
            return

        self._frames_seen += 1
        if self._frames_seen == 1:
            self.get_logger().info(f'First camera frame received ({msg.width}x{msg.height}).')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge failed: {exc}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray)

        debug_img = cv_image.copy() if self.publish_debug else None
        if debug_img is not None and ids is not None and len(ids) > 0:
            aruco.drawDetectedMarkers(debug_img, corners, ids)

        found = False
        if ids is not None and len(ids) > 0:
            for i, marker_id in enumerate(ids.flatten()):
                if int(marker_id) != self.target_marker_id:
                    continue
                found = self._handle_marker(msg, corners[i][0], int(marker_id), debug_img)
                break

        self.detected_pub.publish(Bool(data=found))

        if debug_img is not None:
            if not found:
                cv2.putText(debug_img, 'searching for dock marker '
                            f'{self.target_marker_id}', (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            try:
                out = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
                out.header = msg.header
                self.debug_image_pub.publish(out)
            except Exception as exc:
                self.get_logger().warn(f'debug image publish failed: {exc}', once=True)

    def _handle_marker(self, msg: Image, marker_corners, marker_id: int, debug_img) -> bool:
        ok, rvec, tvec = cv2.solvePnP(
            self.marker_3d_corners, marker_corners,
            self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            ok, rvec, tvec = cv2.solvePnP(
                self.marker_3d_corners, marker_corners,
                self.camera_matrix, self.dist_coeffs)
        if not ok:
            return False

        R, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R)
        tx, ty, tz = float(tvec[0]), float(tvec[1]), float(tvec[2])

        stamp = msg.header.stamp
        optical_frame = msg.header.frame_id or self.camera_optical_frame

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = optical_frame
        tf_msg.child_frame_id = f'{self.marker_frame_prefix}_{marker_id}'
        tf_msg.transform.translation.x = tx
        tf_msg.transform.translation.y = ty
        tf_msg.transform.translation.z = tz
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

        pose_cam = PoseStamped()
        pose_cam.header.stamp = stamp
        pose_cam.header.frame_id = optical_frame
        pose_cam.pose.position.x = tx
        pose_cam.pose.position.y = ty
        pose_cam.pose.position.z = tz
        pose_cam.pose.orientation.x = qx
        pose_cam.pose.orientation.y = qy
        pose_cam.pose.orientation.z = qz
        pose_cam.pose.orientation.w = qw

        published = False
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, optical_frame, rclpy.time.Time())
            pose_base = tf2_geometry_msgs.do_transform_pose_stamped(pose_cam, transform)
            pose_base.header.stamp = stamp
            pose_base.header.frame_id = self.base_frame
            self.marker_pose_pub.publish(pose_base)
            published = True
        except Exception as exc:
            # Deliberately no fallback publish: an optical-frame pose on this
            # topic would be interpreted as x=forward and drive the robot sideways.
            self.get_logger().warn(
                f'No transform {optical_frame} -> {self.base_frame} yet ({exc}); '
                'marker pose withheld.', throttle_duration_sec=2.0)

        if debug_img is not None:
            cv2.drawFrameAxes(debug_img, self.camera_matrix, self.dist_coeffs,
                              rvec, tvec, self.marker_size * 0.75)
            dist = math.sqrt(tx * tx + ty * ty + tz * tz)
            cv2.putText(debug_img, f'dock {marker_id}  {dist:.2f} m',
                        (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return published


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
