"""All-ID ArUco observations for exploration, independent of dock ID 42."""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


def dictionary_for(name):
    if not name.startswith('DICT_') or not hasattr(cv2.aruco, name):
        raise ValueError('Unknown ArUco dictionary: ' + name)
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def observations(image, matrix, distortion, dictionary, marker_size,
                 max_reprojection_error=3.0):
    """Return (id, translation, quaternion) for calibrated, valid detections."""
    parameters = (cv2.aruco.DetectorParameters_create()
                  if hasattr(cv2.aruco, 'DetectorParameters_create')
                  else cv2.aruco.DetectorParameters())
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    if hasattr(cv2.aruco, 'ArucoDetector'):
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, parameters).detectMarkers(image)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary, parameters=parameters)
    if ids is None:
        return []
    half = marker_size / 2.0
    model = np.array([[-half, half, 0], [half, half, 0],
                      [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
    result = []
    for marker_id, pixels in zip(ids.ravel(), corners):
        ok, rotation, translation = cv2.solvePnP(
            model, pixels.reshape(4, 2), matrix, distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if (not ok or not np.isfinite(translation).all() or not np.isfinite(rotation).all()
                or translation[2, 0] <= 0):
            continue
        reprojection, _ = cv2.projectPoints(model, rotation, translation, matrix, distortion)
        error = np.sqrt(np.mean(np.sum((reprojection.reshape(4, 2)
                                       - pixels.reshape(4, 2)) ** 2, axis=1)))
        if not np.isfinite(error) or error > max_reprojection_error:
            continue
        angle = float(np.linalg.norm(rotation))
        xyz = rotation.ravel() * (math.sin(angle / 2) / angle if angle > 1e-12 else 0.5)
        quaternion = (*map(float, xyz), math.cos(angle / 2))
        result.append((int(marker_id), tuple(map(float, translation.ravel())), quaternion))
    return result


class ArucoSearchDetector(Node):
    def __init__(self, **node_options):
        super().__init__('aruco_search_detector', **node_options)
        self.declare_parameter('dictionary_name', 'DICT_5X5_250')
        self.declare_parameter('marker_size', 0.15)
        self.declare_parameter('camera_image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('max_detection_rate', 5.0)
        self.declare_parameter('max_reprojection_error', 3.0)
        # Exploration runs with the docking pipeline switched off, so
        # /docking/debug_image does not exist during a search. Without this
        # there is no way to see whether the camera is looking at anything,
        # and a search that finds nothing is indistinguishable from a detector
        # that is not working.
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_topic', '/exploration/debug_image')
        self.dictionary_name = self.get_parameter('dictionary_name').value
        self.dictionary = dictionary_for(self.dictionary_name)
        self.size = float(self.get_parameter('marker_size').value)
        rate = float(self.get_parameter('max_detection_rate').value)
        if self.size <= 0 or rate <= 0:
            raise ValueError('marker_size and max_detection_rate must be positive')
        self.period = 1.0 / rate
        self.last_frame = None
        self.matrix = self.distortion = None
        self.bridge = CvBridge()
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)
        self.debug_pub = (self.create_publisher(
            Image, self.get_parameter('debug_image_topic').value, 2)
            if self.publish_debug else None)
        self.pub = self.create_publisher(MarkerArray, '/exploration/detections', 5)
        self.heartbeat = self.create_publisher(Header, '/exploration/camera_stamp', 5)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value,
                                 self._info, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('camera_image_topic').value,
                                 self._image, qos_profile_sensor_data)

    def _info(self, msg):
        if msg.k[0] > 0 and msg.k[4] > 0:
            self.matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.distortion = np.array(msg.d, dtype=np.float64)

    def _publish_debug(self, msg, gray, found):
        """Annotated view of what the search camera is looking at."""
        if self.debug_pub is None:
            return
        canvas = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        for marker_id, position, _ in found:
            distance = math.sqrt(sum(v * v for v in position))
            cv2.putText(canvas, f'id {marker_id}  {distance:.2f} m', (8, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if not found:
            cv2.putText(canvas, 'searching, no marker in view', (8, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 160, 255), 2)
        out = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        out.header = msg.header
        self.debug_pub.publish(out)

    def _image(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if (self.last_frame is not None and 0 <= now - self.last_frame < self.period):
            return
        if self.matrix is None or not msg.header.frame_id:
            return
        self.last_frame = now
        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            found = observations(gray, self.matrix, self.distortion, self.dictionary,
                                 self.size, self.get_parameter('max_reprojection_error').value)
            self._publish_debug(msg, gray, found)
        except (cv2.error, CvBridgeError, ValueError, TypeError) as exc:
            self.get_logger().warn(f'Cannot process camera frame: {exc}',
                                   throttle_duration_sec=5.0)
            return
        output = MarkerArray()
        for marker_id, position, quaternion in found:
            marker = Marker()
            marker.header = msg.header
            marker.ns = self.dictionary_name
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = position
            (marker.pose.orientation.x, marker.pose.orientation.y,
             marker.pose.orientation.z, marker.pose.orientation.w) = quaternion
            marker.scale.x = marker.scale.y = self.size
            marker.scale.z = 0.005
            marker.color.g = marker.color.a = 1.0
            marker.lifetime.sec = 1
            output.markers.append(marker)
        self.pub.publish(output)
        self.heartbeat.publish(msg.header)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoSearchDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
