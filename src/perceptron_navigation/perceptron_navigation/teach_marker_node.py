#!/usr/bin/env python3
"""Survey a wall board into the map frame by looking at it from a localised robot.

    T_map_board = T_map_base . T_base_board

The right-hand side is the same measurement aruco_localizer_node makes, run
backwards: instead of trusting the board's map pose to find the robot, trust
the robot's map pose to find the board.

HOW TO USE IT

  1. Localise the robot properly FIRST -- AMCL against the saved map, a "2D
     Pose Estimate" in RViz, then drive a loop until the particle cloud is
     tight. Everything below inherits this error, so it is worth the minute.
  2. Park facing the board, close (well under a metre for 60 mm tiles).
  3. ros2 service call /aruco/teach/capture std_srvs/srv/Trigger
  4. Move to a different viewpoint -- off to one side, nearer, further -- and
     capture again. Three or four viewpoints, not three or four captures from
     the same spot.
  5. ros2 service call /aruco/teach/save std_srvs/srv/Trigger

WHY MULTIPLE VIEWPOINTS AND NOT JUST MORE SAMPLES

Averaging kills zero-mean noise, and repeated samples from one pose are not
zero-mean: a head-on view is the worst-conditioned view of a planar target,
and its error is a bias, not a wobble. Moving the robot changes the sign of
that bias, so a handful of spread-out viewpoints beats a thousand samples from
one spot. The spread reported at save time is the honest error bar -- if it is
large, the survey is bad and no amount of extra captures will fix it.

WHAT IS WRITTEN

A `boards:` entry in marker_map.yaml, in exactly the form marker_map.py
validates: x, y, z and yaw_deg, with yaw_deg the map direction the board face
points. The board is assumed flat on a vertical wall and upright; a measured
tilt is discarded, and the residual it leaves shows up in the spread.
"""

import math
import os
import re
import threading
from collections import deque

import numpy as np
import rclpy
import tf2_ros
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger

import msgpack
import zmq

from perceptron_navigation import marker_map as mm
from perceptron_navigation.aruco_localizer_node import transform_to_matrix

SLOTS = ('top_left', 'top_right', 'bottom_left', 'bottom_right')


def parse_ids(value):
    """'0,1,2,3' or [0,1,2,3] -> [0, 1, 2, 3]."""
    if isinstance(value, str):
        tokens = [t for t in re.split(r'[\s,\[\]]+', value) if t]
    else:
        tokens = list(value)
    try:
        return [int(t) for t in tokens]
    except (TypeError, ValueError):
        raise ValueError('cannot read marker ids from %r' % (value,))


class TeachMarkerNode(Node):

    def __init__(self):
        super().__init__('teach_marker_node')

        self.declare_parameter('jetson_ip', '192.168.1.6')
        self.declare_parameter('telemetry_port', 5555)
        self.declare_parameter('output_path',
                               '~/perceptron_test_ws/config/marker_map.yaml')
        self.declare_parameter('map_reference', '')
        self.declare_parameter('board_name', 'board_a')
        self.declare_parameter('tile_size', 0.06)
        self.declare_parameter('tile_spacing', 0.08)
        # Slot order is top_left, top_right, bottom_left, bottom_right, as seen
        # by someone standing in front of the wall reading the markers.
        # A comma-separated string rather than an integer array: ROS 2 launch
        # substitutions are strings, and handing one to an int-array parameter
        # is a type error at node construction rather than anything readable.
        self.declare_parameter('ids', '0,1,2,3')

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
        self.declare_parameter('samples_per_capture', 20)
        self.declare_parameter('capture_timeout', 15.0)
        self.declare_parameter('max_reprojection_error', 2.0)
        self.declare_parameter('min_tiles', 4)

        get = self.get_parameter
        self.output_path = os.path.expanduser(get('output_path').value)
        self.map_reference = get('map_reference').value
        self.board_name = get('board_name').value
        self.tile_size = float(get('tile_size').value)
        self.tile_spacing = float(get('tile_spacing').value)
        self.map_frame = get('map_frame').value
        self.base_frame = get('base_frame').value
        self.camera_optical_frame = get('camera_optical_frame').value
        self.samples_per_capture = int(get('samples_per_capture').value)
        self.capture_timeout = float(get('capture_timeout').value)
        self.max_reprojection_error = float(get('max_reprojection_error').value)
        self.min_tiles = int(get('min_tiles').value)

        ids = parse_ids(get('ids').value)
        if len(ids) != 4:
            raise ValueError('ids must list exactly 4 marker ids, got %r' % ids)
        if len(set(ids)) != 4:
            raise ValueError('ids must be distinct, got %r' % ids)
        self.id_slots = dict(zip(ids, SLOTS))
        # A board dict of exactly the shape marker_map.py expects, with a
        # placeholder pose -- only its geometry is used while surveying.
        self.board = {
            'tile_size': self.tile_size,
            'tile_spacing': self.tile_spacing,
            'ids': self.id_slots,
            'pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw_deg': 0.0},
        }

        self.calibrated_size = (int(get('calibration_width').value),
                                int(get('calibration_height').value))
        self.base_camera_matrix = np.array(
            get('camera_matrix').value, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(get('dist_coeffs').value, dtype=np.float64)

        # Each row is (x, y, z, yaw, range, reproj, robot_x, robot_y, robot_yaw).
        # Only the first four are the measurement; the rest are diagnostics.
        self.samples = []
        self.viewpoint_means = []
        self.viewpoints = 0
        self.capturing = False
        self.capture_buffer = []
        self.capture_deadline = None
        self.capture_reject = ''

        self.inbox = deque(maxlen=5)
        self.inbox_lock = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_service(Trigger, '/aruco/teach/capture', self._capture_cb)
        self.create_service(Trigger, '/aruco/teach/save', self._save_cb)
        self.create_service(Trigger, '/aruco/teach/reset', self._reset_cb)

        self.running = True
        self.jetson_url = 'tcp://%s:%d' % (get('jetson_ip').value,
                                           int(get('telemetry_port').value))
        threading.Thread(target=self._zmq_worker, daemon=True).start()
        self.create_timer(0.05, self._process_inbox)

        self.get_logger().info(
            'Teaching board "%s" (%d mm tiles, %d mm spacing), ids %s -> %s.\n'
            '  Localise the robot FIRST, then:\n'
            '    ros2 service call /aruco/teach/capture std_srvs/srv/Trigger\n'
            '  from 3-4 different viewpoints, then:\n'
            '    ros2 service call /aruco/teach/save std_srvs/srv/Trigger'
            % (self.board_name, int(self.tile_size * 1000),
               int(self.tile_spacing * 1000), ids, list(SLOTS)))

    def destroy_node(self):
        self.running = False
        super().destroy_node()

    # ----------------------------------------------------------------- input

    def _zmq_worker(self):
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, b'aruco')
        socket.setsockopt(zmq.RCVTIMEO, 1000)
        socket.set_hwm(5)
        socket.connect(self.jetson_url)
        self.get_logger().info('Subscribed to ArUco corners at %s' % self.jetson_url)
        while self.running and rclpy.ok():
            try:
                frames = socket.recv_multipart()
            except zmq.Again:
                continue
            except Exception:
                continue
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

    def _process_inbox(self):
        if not self.capturing:
            return
        with self.inbox_lock:
            payload = self.inbox.pop() if self.inbox else None
            self.inbox.clear()

        if payload is not None:
            sample = self._board_pose_in_map(payload)
            if sample is not None:
                self.capture_buffer.append(sample)

        if len(self.capture_buffer) >= self.samples_per_capture:
            self._finish_capture(True)
        elif self.get_clock().now() > self.capture_deadline:
            self._finish_capture(False)

    # ------------------------------------------------------------ the measure

    def _board_pose_in_map(self, payload):
        """One observation -> (x, y, z, yaw) of the board in the map frame."""
        ids = payload.get('ids') or []
        known = [i for i in ids if i in self.id_slots]
        if len(known) < self.min_tiles:
            self.capture_reject = ('saw %d of %d tiles -- get closer, or drop '
                                   'min_tiles' % (len(known), len(self.id_slots)))
            return None

        corners_by_id = dict(zip(ids, payload.get('corners') or []))
        object_points, image_points, used = mm.assemble_correspondences(
            self.board, known, [corners_by_id[i] for i in known])
        if not used:
            return None

        width = int(payload.get('width', self.calibrated_size[0]))
        height = int(payload.get('height', self.calibrated_size[1]))
        camera_matrix = mm.scale_intrinsics(
            self.base_camera_matrix, self.calibrated_size, (width, height))
        camera_from_board, error = mm.solve_board_pose(
            object_points, image_points, camera_matrix, self.dist_coeffs)
        if camera_from_board is None:
            self.capture_reject = 'PnP failed'
            return None
        if error > self.max_reprojection_error:
            self.capture_reject = ('reprojection %.2f px > %.2f'
                                   % (error, self.max_reprojection_error))
            return None

        try:
            base_from_camera = transform_to_matrix(self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_optical_frame, rclpy.time.Time()))
            map_from_base = transform_to_matrix(self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()))
        except Exception as exc:
            self.capture_reject = ('TF unavailable (%s). Is AMCL running and '
                                   'localised?' % exc)
            return None

        map_from_board = map_from_base.dot(base_from_camera).dot(camera_from_board)
        distance = float(np.linalg.norm(camera_from_board[:3, 3]))
        robot_yaw = math.atan2(float(map_from_base[1, 0]), float(map_from_base[0, 0]))
        # The board's +Z axis in map coordinates is its face normal; flattened
        # into the floor plane it is exactly the yaw the YAML wants.
        normal = map_from_board[:3, 2]
        if math.hypot(normal[0], normal[1]) < 1e-6:
            self.capture_reject = 'board normal is vertical -- is it on a wall?'
            return None
        return (float(map_from_board[0, 3]), float(map_from_board[1, 3]),
                float(map_from_board[2, 3]),
                math.atan2(float(normal[1]), float(normal[0])),
                distance, float(error),
                float(map_from_base[0, 3]), float(map_from_base[1, 3]), robot_yaw)

    # -------------------------------------------------------------- services

    def _capture_cb(self, request, response):
        if self.capturing:
            response.success = False
            response.message = 'already capturing'
            return response
        self.capturing = True
        self.capture_buffer = []
        self.capture_reject = ''
        self.capture_deadline = self.get_clock().now() + rclpy.duration.Duration(
            seconds=self.capture_timeout)
        response.success = True
        response.message = ('Capturing %d samples (timeout %.0fs). Hold still.'
                            % (self.samples_per_capture, self.capture_timeout))
        self.get_logger().info(response.message)
        return response

    def _finish_capture(self, complete):
        self.capturing = False
        count = len(self.capture_buffer)
        if count == 0:
            self.get_logger().warn(
                'Capture got nothing. Last reason: %s'
                % (self.capture_reject or 'no observations arrived at all'))
            return
        self.samples.extend(self.capture_buffer)
        self.viewpoints += 1
        x, y, z, yaw = self._summarise(self.capture_buffer)
        self.viewpoint_means.append((x, y, z, yaw))

        block = np.asarray(self.capture_buffer, dtype=np.float64)
        rng, reproj = float(np.mean(block[:, 4])), float(np.mean(block[:, 5]))
        rx, ry = float(np.mean(block[:, 6])), float(np.mean(block[:, 7]))
        ryaw = math.atan2(float(np.mean(np.sin(block[:, 8]))),
                          float(np.mean(np.cos(block[:, 8]))))

        self.get_logger().info(
            'Viewpoint %d: %d samples%s\n'
            '    board  -> x=%.3f y=%.3f z=%.3f yaw=%.1f deg\n'
            '    seen from range %.2f m, reprojection %.2f px\n'
            '    robot was at x=%.3f y=%.3f yaw=%.1f deg  (per AMCL)'
            % (self.viewpoints, count, '' if complete else ' (timed out)',
               x, y, z, math.degrees(yaw), rng, reproj,
               rx, ry, math.degrees(ryaw)))

        # Spread ACROSS viewpoints is the number that matters, and seeing it
        # grow now beats discovering it at save time.
        if len(self.viewpoint_means) > 1:
            means = np.asarray(self.viewpoint_means, dtype=np.float64)
            cx, cy = float(np.mean(means[:, 0])), float(np.mean(means[:, 1]))
            worst = float(np.max(np.hypot(means[:, 0] - cx, means[:, 1] - cy)))
            verdict = ('GOOD' if worst < 0.03 else
                       'usable' if worst < 0.06 else 'TOO LARGE -- do not save')
            self.get_logger().info(
                '    viewpoint disagreement so far: %.3f m  [%s]'
                % (worst, verdict))
            if worst >= 0.06:
                self.get_logger().warn(
                    'Viewpoints disagree by %.3f m. The board has not moved, so '
                    'this is one of: (a) AMCL was wrong between captures -- '
                    'check the robot poses logged above against where it really '
                    'was; (b) tile_size/tile_spacing are wrong -- a scale error '
                    'turns into a range error proportional to distance; or (c) '
                    'the range above is too large for 60 mm tiles. Call '
                    '/aruco/teach/reset and start over once fixed.' % worst)

    @staticmethod
    def _summarise(samples):
        # Columns beyond the first four are diagnostics and are ignored here.
        array = np.asarray(samples, dtype=np.float64)
        return (float(np.mean(array[:, 0])), float(np.mean(array[:, 1])),
                float(np.mean(array[:, 2])),
                math.atan2(float(np.mean(np.sin(array[:, 3]))),
                           float(np.mean(np.cos(array[:, 3])))))

    def _spread(self, x, y, yaw):
        array = np.asarray(self.samples, dtype=np.float64)
        position = np.hypot(array[:, 0] - x, array[:, 1] - y)
        yaw_error = np.arctan2(np.sin(array[:, 3] - yaw), np.cos(array[:, 3] - yaw))
        return float(np.max(position)), float(np.max(np.abs(yaw_error)))

    def _reset_cb(self, request, response):
        self.samples = []
        self.viewpoint_means = []
        self.viewpoints = 0
        response.success = True
        response.message = 'Cleared all samples.'
        self.get_logger().info(response.message)
        return response

    def _save_cb(self, request, response):
        if not self.samples:
            response.success = False
            response.message = 'Nothing captured yet.'
            return response

        x, y, z, yaw = self._summarise(self.samples)
        position_spread, yaw_spread = self._spread(x, y, yaw)

        entry = {
            'tile_size': round(self.tile_size, 4),
            'tile_spacing': round(self.tile_spacing, 4),
            'ids': {int(k): v for k, v in self.id_slots.items()},
            'pose': {'x': round(x, 4), 'y': round(y, 4), 'z': round(z, 4),
                     'yaw_deg': round(math.degrees(yaw), 2)},
            'surveyed': {
                'viewpoints': self.viewpoints,
                'samples': len(self.samples),
                'max_position_spread_m': round(position_spread, 4),
                'max_yaw_spread_deg': round(math.degrees(yaw_spread), 2),
            },
        }

        document = self._read_existing()
        document['boards'][self.board_name] = entry
        if self.map_reference:
            document['map_reference'] = self.map_reference

        # Atomic replace: a half-written marker map is worse than none,
        # because the robot would localise confidently against garbage.
        directory = os.path.dirname(self.output_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        temporary = self.output_path + '.tmp'
        with open(temporary, 'w') as handle:
            yaml.safe_dump(document, handle, default_flow_style=False, sort_keys=False)
        os.replace(temporary, self.output_path)

        warning = ''
        if self.viewpoints < 3:
            warning = (' WARNING: only %d viewpoint(s). A head-on view is the '
                       'worst-conditioned view of a planar board and its error '
                       'is a bias, not noise -- capture from a few different '
                       'positions.' % self.viewpoints)
        if position_spread > 0.05 or yaw_spread > math.radians(5.0):
            warning += (' WARNING: spread is large (%.3f m, %.1f deg); the robot '
                        'was probably not well localised while surveying.'
                        % (position_spread, math.degrees(yaw_spread)))

        response.success = True
        response.message = (
            'Wrote board "%s" to %s: x=%.3f y=%.3f z=%.3f yaw=%.1f deg from %d '
            'samples over %d viewpoint(s); spread %.3f m / %.1f deg.%s'
            % (self.board_name, self.output_path, x, y, z, math.degrees(yaw),
               len(self.samples), self.viewpoints, position_spread,
               math.degrees(yaw_spread), warning))
        self.get_logger().info(response.message)
        return response

    def _read_existing(self):
        """Merge into an existing map so several boards can be taught in turn."""
        try:
            with open(self.output_path, 'r') as handle:
                document = yaml.safe_load(handle)
            if isinstance(document, dict) and isinstance(document.get('boards'), dict):
                return document
        except (OSError, ValueError, yaml.YAMLError):
            pass
        return {'frame_id': self.map_frame,
                'map_reference': self.map_reference or '',
                'boards': {}}


def main(args=None):
    rclpy.init(args=args)
    node = TeachMarkerNode()
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
