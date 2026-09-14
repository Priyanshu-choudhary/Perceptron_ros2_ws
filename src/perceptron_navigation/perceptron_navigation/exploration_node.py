"""Autonomous frontier exploration and camera-viewpoint ArUco search.

Motion is exclusively through Nav2 NavigateToPose and collision-checked Spin.
An accepted action is cancelled and acknowledged before another is sent or a
terminal result is reported. A late goal acceptance is cancelled as well.
"""

from datetime import datetime, timezone
import json
import math
import os
import time
import uuid

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import BackUp, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String
from std_srvs.srv import SetBool, Trigger
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener, TransformException

from perceptron_navigation.aruco_search_detector import dictionary_for
from perceptron_navigation.exploration_storage import save_run
from perceptron_navigation.frontier_planner import Grid, choose_target


TERMINAL = {'IDLE', 'FOUND', 'COMPLETE', 'NOT_FOUND', 'EXHAUSTED', 'FAILED',
            'CANCELLED', 'STOP_UNCONFIRMED'}


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class ExplorationNode(Node):
    def __init__(self, **node_options):
        super().__init__('exploration_node', **node_options)
        defaults = {
            'map_topic': '/map', 'map_frame': 'map', 'base_frame': 'base_footprint',
            'scan_topic': '/scan', 'nav_profile': 'dwb',
            'dictionary_name': 'DICT_5X5_250', 'marker_size': 0.15,
            'auto_start': False, 'robot_radius': 0.33, 'clearance_margin': 0.05,
            'frontier_standoff': 0.65, 'min_frontier_cells': 5,
            'min_goal_distance': 0.35, 'viewpoint_spacing': 1.2,
            'frontier_gain_weight': 2.5, 'seed_search_radius': 1.0,
            'max_clearance_recoveries': 3, 'recovery_backup_distance': 0.25,
            'max_duration': 900.0, 'max_goals': 150, 'goal_timeout': 120.0,
            'startup_timeout': 120.0, 'sensor_timeout': 3.0, 'map_timeout': 12.0,
            'action_ack_timeout': 15.0, 'cancel_timeout': 10.0,
            'settle_seconds': 3.0, 'failed_goal_cooldown': 120.0,
            'visited_goal_cooldown': 60.0, 'goal_exclusion_radius': 0.45,
            'marker_confirmations': 3, 'marker_consistency_distance': 0.30,
            'marker_transform_timeout': 0.15,
            'output_directory': '~/.ros/perceptron_exploration',
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
        self.declare_parameter('target_marker_id', -1)
        self.dictionary_size = len(dictionary_for(self.p('dictionary_name')).bytesList)
        if self.p('nav_profile') not in ('dwb', 'mppi'):
            raise ValueError('nav_profile must be dwb or mppi')
        for key in ('robot_radius', 'frontier_standoff', 'viewpoint_spacing', 'max_duration',
                    'goal_timeout', 'sensor_timeout', 'map_timeout', 'startup_timeout',
                    'max_goals', 'marker_confirmations', 'action_ack_timeout', 'cancel_timeout'):
            if self.p(key) <= 0:
                raise ValueError(key + ' must be positive')
        if not -1 <= self.p('target_marker_id') < self.dictionary_size:
            raise ValueError('target_marker_id outside selected dictionary')
        self.add_on_set_parameters_callback(self._validate_parameters)
        self.state, self.detail = 'IDLE', 'Use explore start or explore search ID'
        self.run_id = ''
        self.target_id = -1
        self.started = 0.0
        self.started_wall = 0.0
        self.grid = None
        self.map_received = None
        self.map_revision = 0
        self.scan_stamp = self.camera_stamp = None
        self.motion = None
        self.pending_terminal = None
        self.cancel_started_wall = None
        self.relay_future = None
        self.relay_hold = False
        self.target = None
        self.excluded = []
        self.scanned = []
        self.failures = 0
        self.goals_sent = 0
        self.sweep_remaining = 0
        self.clearance_recoveries = 0
        self.settle_until = 0.0
        self.empty_revisions = []
        self.found = None
        self.confirmation = None
        self.result_file = None
        self.last_status = None
        self.foreign_active = set()
        # node= matters: without it the buffer's timeout path uses the system
        # clock while every stamp here is simulation time.
        self.tf_buffer = Buffer(node=self)
        # spin_thread=True gives the listener its own executor thread. Without
        # it a blocking lookup_transform below would deadlock: this node runs on
        # a single-threaded spin, so waiting for a transform inside a callback
        # would block the very thread that has to receive it.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/exploration/status', durable)
        self.goal_pub = self.create_publisher(PoseStamped, '/exploration/goal', durable)
        self.found_pub = self.create_publisher(PoseStamped, '/exploration/found_marker', durable)
        self.create_subscription(OccupancyGrid, self.p('map_topic'), self._map, durable)
        self.create_subscription(LaserScan, self.p('scan_topic'),
                                 lambda msg: setattr(self, 'scan_stamp', seconds(msg.header.stamp)),
                                 qos_profile_sensor_data)
        from visualization_msgs.msg import MarkerArray
        self.create_subscription(MarkerArray, '/exploration/detections', self._markers, 5)
        self.create_subscription(Header, '/exploration/camera_stamp',
                                 lambda msg: setattr(self, 'camera_stamp', seconds(msg.stamp)), 5)
        self.create_subscription(String, '/mission/status',
                                 lambda msg: self._foreign('mission', msg.data), 10)
        self.create_subscription(String, '/docking/status',
                                 lambda msg: self._foreign('docking', msg.data), 10)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.spin = ActionClient(self, Spin, '/spin')
        self.backup = ActionClient(self, BackUp, '/backup')
        self.relay = self.create_client(SetBool, '/cmd_vel_relay/enable')
        self.lifecycle = {
            name: {'client': self.create_client(GetState, '/' + name + '/get_state'),
                   'future': None, 'active': False, 'checked': -float('inf')}
            for name in ('bt_navigator', 'controller_server', 'behavior_server')
        }
        self.create_service(Trigger, '/exploration/start', self._start)
        self.create_service(Trigger, '/exploration/cancel', self._cancel)
        self.create_timer(0.5, self._tick)
        self.auto_pending = bool(self.p('auto_start'))
        self._publish_status()

    def p(self, name):
        return self.get_parameter(name).value

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def active(self):
        return self.state not in TERMINAL

    def _validate_parameters(self, params):
        for param in params:
            if param.name == 'target_marker_id':
                if self.active() or self.motion is not None:
                    return SetParametersResult(successful=False, reason='Cancel the current run first')
                if type(param.value) is not int or not -1 <= param.value < self.dictionary_size:
                    return SetParametersResult(successful=False, reason='ID outside selected dictionary')
        return SetParametersResult(successful=True)

    def _foreign(self, owner, text):
        state = text.split(' |')[0]
        idle = {'IDLE', 'DONE', 'FAILED', 'CANCELLED', 'DOCKED'}
        if state in idle:
            self.foreign_active.discard(owner)
        else:
            self.foreign_active.add(owner)
            if self.active():
                self._stop('FAILED', owner + ' became active; exploration is yielding control')

    def _map(self, msg):
        if msg.header.frame_id != self.p('map_frame'):
            self.get_logger().warn('Ignoring occupancy grid in unexpected frame', once=True)
            return
        if (msg.info.resolution <= 0 or msg.info.width * msg.info.height != len(msg.data)
                or not len(msg.data)):
            return
        origin = msg.info.origin
        self.grid = Grid(np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width).copy(), msg.info.resolution,
            origin.position.x, origin.position.y, yaw_of(origin.orientation))
        self.map_received = self.now()
        self.map_revision += 1

    def _robot(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.p('map_frame'), self.p('base_frame'), Time())
            age = self.now() - seconds(tf.header.stamp)
            if not 0 <= age <= self.p('sensor_timeout'):
                return None
            return (tf.transform.translation.x, tf.transform.translation.y)
        except TransformException:
            return None

    def _fresh(self, stamp):
        return stamp is not None and -0.1 <= self.now() - stamp <= self.p('sensor_timeout')

    def _ready_problem(self):
        if self.grid is None or self.map_received is None:
            return 'waiting for /map'
        if self.now() - self.map_received > self.p('map_timeout'):
            return 'map updates stopped'
        if not self._fresh(self.scan_stamp):
            return 'laser scan missing or stale'
        if self._robot() is None:
            return 'map-to-robot transform missing or stale'
        if not self.nav.server_is_ready():
            return 'Nav2 NavigateToPose server unavailable'
        for name, entry in self.lifecycle.items():
            if name == 'behavior_server' and self.target_id < 0:
                continue
            future = entry['future']
            if future is not None and future.done():
                try:
                    entry['active'] = future.result().current_state.id == 3
                except Exception:
                    entry['active'] = False
                entry['future'] = None
                entry['checked'] = time.monotonic()
            if (entry['future'] is None and time.monotonic() - entry['checked'] > 1.0
                    and entry['client'].service_is_ready()):
                entry['future'] = entry['client'].call_async(GetState.Request())
            if not entry['active'] or time.monotonic() - entry['checked'] > 5.0:
                return 'waiting for active Nav2 lifecycle: ' + name
        if self.target_id >= 0:
            if not self._fresh(self.camera_stamp):
                return 'calibrated camera detections missing or stale'
            if not self.spin.server_is_ready():
                return 'Nav2 Spin server unavailable'
        return None

    def _start(self, request, response):
        if self.active() or self.motion is not None or self.relay_hold:
            response.success = False
            response.message = 'Run active or motion stop unconfirmed; cancel/resolve it first'
            return response
        if self.foreign_active:
            response.success = False
            response.message = 'Another controller is active: ' + ', '.join(self.foreign_active)
            return response
        self.target_id = int(self.p('target_marker_id'))
        self.run_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ-') + uuid.uuid4().hex[:8]
        self.started, self.started_wall = self.now(), time.monotonic()
        self.excluded, self.scanned, self.empty_revisions = [], [], []
        self.failures = self.goals_sent = self.sweep_remaining = 0
        self.clearance_recoveries = 0
        self.found = self.confirmation = self.result_file = None
        self.pending_terminal = self.cancel_started_wall = self.relay_future = None
        self.target = None
        self._state('WAITING', 'Waiting for SLAM, sensors and Nav2')
        response.success = True
        response.message = json.dumps({'run_id': self.run_id, 'target_marker_id': self.target_id})
        return response

    def _cancel(self, request, response):
        self.auto_pending = False
        if self.active():
            self._stop('CANCELLED', 'Cancelled by user')
        response.success = True
        response.message = 'Cancellation requested' if self.motion else 'No motion active'
        return response

    def _state(self, state, detail):
        self.state, self.detail = state, detail
        self._publish_status()

    def _publish_status(self):
        payload = json.dumps({'run_id': self.run_id, 'state': self.state, 'detail': self.detail,
                              'target_marker_id': self.target_id, 'goals_sent': self.goals_sent,
                              'result_file': self.result_file})
        self.status_pub.publish(String(data=payload))
        if payload != self.last_status:
            self.get_logger().info(f'{self.state}: {self.detail}')
            self.last_status = payload

    def _marker_transform(self, marker):
        """map <- the camera frame at a detection's timestamp, or None.

        Asking for the exact stamp fails essentially always: the camera runs at
        30 Hz and the transform chain is assembled a few milliseconds behind it,
        so the image is stamped AHEAD of the newest transform and tf2 refuses to
        extrapolate into the future. Measured, parked in front of the marker,
        every single lookup failed by 5 to 25 ms and the search never confirmed
        anything while the detector was publishing id 42 at 4 Hz.

        So: ask for the exact time, and if that is not yet available fall back to
        the latest transform, provided it is recent enough that using it cannot
        move the observation meaningfully. At a sweep rate of 0.6 rad/s a 25 ms
        skew turns into about one degree, which is centimetres at dock range and
        far inside marker_consistency_distance.
        """
        stamp = seconds(marker.header.stamp)
        target, source = self.p('map_frame'), marker.header.frame_id
        try:
            return self.tf_buffer.lookup_transform(
                target, source, Time.from_msg(marker.header.stamp),
                timeout=Duration(seconds=self.p('marker_transform_timeout')))
        except TransformException:
            pass
        tf = self.tf_buffer.lookup_transform(target, source, Time())
        skew = abs(stamp - seconds(tf.header.stamp))
        if skew > self.p('marker_transform_timeout'):
            self.get_logger().warn(
                f'Marker {marker.id} discarded: newest {source} -> {target} is '
                f'{skew:.3f} s from the detection', throttle_duration_sec=5.0)
            return None
        return tf

    def _markers(self, msg):
        if not self.active() or self.target_id < 0 or self.pending_terminal or self.grid is None:
            return
        matching = [m for m in msg.markers if m.id == self.target_id
                    and m.ns == self.p('dictionary_name')]
        if not matching:
            # Say WHY a detection was discarded. Every rejection below used to be
            # a bare return, so a search that never confirmed looked exactly like
            # a camera that never saw anything.
            if msg.markers:
                self.get_logger().info(
                    'Ignoring detections ' +
                    ', '.join(f'{m.ns}:{m.id}' for m in msg.markers) +
                    f' (want {self.p("dictionary_name")}:{self.target_id})',
                    throttle_duration_sec=5.0)
            self.confirmation = None
            return
        marker = matching[0]
        stamp = seconds(marker.header.stamp)
        if stamp < self.started or not self._fresh(stamp):
            self.get_logger().warn(
                f'Marker {marker.id} seen but its stamp is unusable '
                f'(stamp {stamp:.2f}, run started {self.started:.2f}, '
                f'now {self.now():.2f})', throttle_duration_sec=5.0)
            return
        observation = PoseStamped(header=marker.header, pose=marker.pose)
        try:
            tf = self._marker_transform(marker)
        except TransformException as exc:
            self.get_logger().warn(
                f'Marker {marker.id} seen but {marker.header.frame_id} -> '
                f'{self.p("map_frame")} lookup failed: {exc}',
                throttle_duration_sec=5.0)
            return
        if tf is None:
            return
        try:
            pose = do_transform_pose_stamped(observation, tf)
        except TransformException as exc:
            self.get_logger().warn(f'Cannot transform marker pose: {exc}',
                                   throttle_duration_sec=5.0)
            return
        xyz = (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
        if not all(math.isfinite(v) for v in xyz):
            return
        count = 1
        if self.confirmation is not None:
            previous_stamp, previous_xyz, previous_count = self.confirmation
            if stamp <= previous_stamp:
                return
            if (stamp - previous_stamp <= self.p('sensor_timeout')
                    and math.dist(xyz, previous_xyz) <= self.p('marker_consistency_distance')):
                count = previous_count + 1
        self.confirmation = stamp, xyz, count
        if count < self.p('marker_confirmations'):
            self.get_logger().info(
                f'Marker {marker.id} confirmation {count}/'
                f'{int(self.p("marker_confirmations"))}',
                throttle_duration_sec=2.0)
            return
        pose.header.frame_id = self.p('map_frame')
        self.found_pub.publish(pose)
        q = pose.pose.orientation
        self.found = {'id': marker.id, 'dictionary': marker.ns,
                      'marker_size': self.p('marker_size'), 'frame': self.p('map_frame'),
                      'stamp': stamp, 'position': dict(zip(('x', 'y', 'z'), xyz)),
                      'orientation': {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w},
                      'confirmations': count}
        self._stop('FOUND', f'Marker {marker.id} confirmed at ({xyz[0]:.2f}, {xyz[1]:.2f})')

    def _send(self, client, goal, kind):
        if self.motion is not None or self.pending_terminal:
            return
        slot = {'kind': kind, 'handle': None, 'cancel_sent': False,
                'sent_wall': time.monotonic(), 'started': self.now()}
        self.motion = slot
        try:
            future = client.send_goal_async(goal)
            future.add_done_callback(lambda f: self._accepted(f, slot))
        except Exception as exc:
            self.motion = None
            self._stop('FAILED', f'Could not send {kind}: {exc}')

    def _accepted(self, future, slot):
        try:
            handle = future.result()
        except Exception as exc:
            if self.motion is slot:
                self.motion = None
                self._stop('FAILED', f'Action request failed: {exc}')
            return
        if self.motion is not slot:
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            return
        if handle is None or not handle.accepted:
            self._motion_done(slot, False)
            return
        slot['handle'] = handle
        handle.get_result_async().add_done_callback(lambda f: self._result(f, slot))
        if self.pending_terminal or slot.get('timed_out'):
            self._cancel_motion()

    def _result(self, future, slot):
        try:
            success = future.result().status == 4
        except Exception:
            success = False
        self._motion_done(slot, success)

    def _motion_done(self, slot, success):
        if self.motion is not slot:
            return
        self.motion = None
        if self.pending_terminal:
            self._finish(*self.pending_terminal)
            return
        success = success and not slot.get('timed_out', False)
        if slot['kind'] == 'navigate':
            cooldown = self.p('visited_goal_cooldown') if success else self.p('failed_goal_cooldown')
            self.excluded.append((self.target.x, self.target.y,
                                  self.p('goal_exclusion_radius'), self.now() + cooldown))
            if not success:
                self.failures += 1
                self._state('PLANNING', 'Goal failed; trying another reachable target')
                return
            self.settle_until = self.now() + self.p('settle_seconds')
            # A four-spin sweep costs roughly ten seconds, so doing one at every
            # goal is only affordable when goals are far apart. Skip it where the
            # camera has already swept nearby: `scanned` is exactly that record.
            self.sweep_remaining = (4 if self.target_id >= 0
                                    and self._needs_sweep() else 0)
            self._state('SETTLING', 'Reached target; waiting for map and camera updates')
        elif slot['kind'] == 'recovery':
            # Either way the map has had a moment to grow; re-plan.
            self._state('PLANNING', 'Recovery finished; re-planning')
        else:
            if not success:
                self.failures += 1
                # Do not mark this viewpoint inspected after a blocked sweep.
                robot = self._robot()
                if robot:
                    self.excluded.append((*robot, self.p('goal_exclusion_radius'),
                                          self.now() + self.p('failed_goal_cooldown')))
                self.sweep_remaining = 0
                self._state('PLANNING', 'Camera sweep blocked; trying another viewpoint')
                return
            self.sweep_remaining -= 1
            if self.sweep_remaining == 0:
                robot = self._robot()
                if robot:
                    self.scanned.append(robot)
                self._state('PLANNING', 'Camera sweep complete')

    def _needs_sweep(self):
        robot = self._robot()
        if robot is None:
            return True
        spacing = self.p('viewpoint_spacing')
        return not any(math.hypot(robot[0] - sx, robot[1] - sy) < spacing
                       for sx, sy in self.scanned)

    def _cancel_motion(self):
        slot = self.motion
        if slot and slot['handle'] is not None and not slot['cancel_sent']:
            slot['cancel_sent'] = True
            try:
                slot['handle'].cancel_goal_async()
            except Exception as exc:
                self.get_logger().error(f'Action cancel request failed: {exc}')

    def _stop(self, state, detail):
        if self.pending_terminal:
            return
        self.pending_terminal = state, detail
        self.cancel_started_wall = time.monotonic()
        if self.motion is not None:
            self._state('CANCELLING', detail + '; waiting for Nav2 to stop')
            self._cancel_motion()
        else:
            self._finish(state, detail)

    def _finish(self, state, detail):
        self.pending_terminal = None
        self._state(state, detail)
        try:
            self.result_file = save_run(self.p('output_directory'), self.run_id, self.grid, {
                'run_id': self.run_id, 'state': state, 'detail': detail,
                'target_marker_id': self.target_id, 'map_frame': self.p('map_frame'),
                'found_marker': self.found, 'goals_sent': self.goals_sent,
                'failed_motions': self.failures, 'camera_viewpoints': self.scanned,
                'elapsed_sim_seconds': max(0.0, self.now() - self.started),
                'coverage_note': 'Reachable sampled viewpoints only; hidden or inaccessible areas may remain.',
            })
        except (OSError, ValueError) as exc:
            self.detail += f'; could not save results: {exc}'
            self.get_logger().error(self.detail)
        self._publish_status()

    def _navigate(self, target):
        self.target = target
        self.goals_sent += 1
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.p('map_frame')
        # Latest TF on every replan, rather than a timestamp which expires.
        goal.pose.pose.position.x, goal.pose.pose.position.y = target.x, target.y
        goal.pose.pose.orientation.z = math.sin(target.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(target.yaw / 2)
        goal.behavior_tree = os.path.join(get_package_share_directory('perceptron_navigation'),
                                         'behavior_trees', 'explore_' + self.p('nav_profile') + '.xml')
        self.goal_pub.publish(goal.pose)
        self._state('NAVIGATING', f'{target.kind}: ({target.x:.2f}, {target.y:.2f})')
        self._send(self.nav, goal, 'navigate')

    def _tick(self):
        self._publish_status()
        if self.auto_pending:
            self.auto_pending = False
            self._start(Trigger.Request(), Trigger.Response())
        if not self.active():
            return
        if self.pending_terminal:
            self._cancel_motion()
            if time.monotonic() - self.cancel_started_wall > self.p('cancel_timeout'):
                # Never claim a stop was confirmed when Nav2 has not replied.
                # Gate the existing relay as an additional stop request.
                self.relay_hold = True
                if self.relay.service_is_ready():
                    self.relay.call_async(SetBool.Request(data=False))
                self._state('STOP_UNCONFIRMED', 'Nav2 stop not acknowledged; relay disable requested. '
                            'Resolve the action/relay state before starting another run.')
            return
        if self.now() - self.started > self.p('max_duration'):
            self._stop('EXHAUSTED', 'Exploration time budget reached')
            return
        if self.state == 'WAITING':
            problem = self._ready_problem()
            if not problem and self.relay_future is None:
                if not self.relay.service_is_ready():
                    problem = 'waiting for velocity relay service'
                else:
                    self.relay_future = self.relay.call_async(SetBool.Request(data=True))
                    problem = 'enabling Nav2 velocity relay'
            elif not problem:
                if not self.relay_future.done():
                    problem = 'waiting for velocity relay acknowledgement'
                else:
                    try:
                        if not self.relay_future.result().success:
                            self._stop('FAILED', 'Velocity relay rejected enable request')
                            return
                    except Exception as exc:
                        self._stop('FAILED', f'Velocity relay failed: {exc}')
                        return
            if problem:
                self.detail = problem
                if time.monotonic() - self.started_wall > self.p('startup_timeout'):
                    self._stop('FAILED', 'Startup timed out: ' + problem)
                return
            self.sweep_remaining = 4 if self.target_id >= 0 else 0
            self._state('PLANNING', 'Ready to explore')
        else:
            problem = self._ready_problem()
            if problem:
                self._stop('FAILED', problem)
                return
        if self.motion is not None:
            slot = self.motion
            if (slot['handle'] is None
                    and time.monotonic() - slot['sent_wall'] > self.p('action_ack_timeout')):
                self._stop('FAILED', 'Nav2 did not acknowledge the action request')
            elif self.now() - slot['started'] > self.p('goal_timeout'):
                if not slot.get('timed_out'):
                    slot['timed_out'] = True
                    slot['timeout_cancel_wall'] = time.monotonic()
                    self.detail = 'Motion timed out; cancelling before trying another target'
                    self._cancel_motion()
                elif time.monotonic() - slot['timeout_cancel_wall'] > self.p('cancel_timeout'):
                    self._stop('FAILED', 'Nav2 did not stop a timed-out motion')
            return
        if self.state == 'SETTLING' and self.now() < self.settle_until:
            return
        if self.sweep_remaining:
            self._state('SCANNING', 'Inspecting surroundings with collision-checked camera sweep')
            goal = Spin.Goal()
            goal.target_yaw = math.pi / 2
            goal.time_allowance.sec = 30
            self._send(self.spin, goal, 'spin')
            return
        if self.goals_sent >= self.p('max_goals'):
            self._stop('EXHAUSTED', 'Exploration goal budget reached')
            return
        robot = self._robot()
        if robot is None:
            return
        self.excluded = [e for e in self.excluded if e[3] > self.now()]
        plan = choose_target(self.grid, robot, excluded=[e[:3] for e in self.excluded],
                             scanned=self.scanned, search=self.target_id >= 0,
                             **{key: self.p(key) for key in (
                                 'robot_radius', 'clearance_margin', 'frontier_standoff',
                                 'min_frontier_cells', 'min_goal_distance', 'viewpoint_spacing',
                                 'frontier_gain_weight', 'seed_search_radius')})
        if plan.reason == 'robot_has_no_clearance':
            # Recoverable, not fatal. The footprint test counts unknown cells as
            # blocking, so this fires whenever the robot is parked against an
            # unmapped pocket - typically right after a sweep near a wall - even
            # though the robot is physically fine. Aborting the whole run there
            # is what ended the measured room_world attempt after five goals.
            # Back out, let the laser fill in, and try again; only give up if it
            # keeps happening.
            self.clearance_recoveries += 1
            if self.clearance_recoveries > self.p('max_clearance_recoveries'):
                self._stop('FAILED', 'Robot is not in sufficiently mapped free space '
                           f'after {self.clearance_recoveries - 1} recoveries; '
                           'check spawn clearance, map alignment and robot_radius')
                return
            self._state('RECOVERING', 'No footprint clearance in the map; backing up')
            goal = BackUp.Goal()
            goal.target.x = -float(self.p('recovery_backup_distance'))
            goal.speed = 0.08
            goal.time_allowance.sec = 20
            self._send(self.backup, goal, 'recovery')
            return
        self.clearance_recoveries = 0
        if plan.target:
            self.empty_revisions = []
            self._navigate(plan.target)
            return
        self._state('PLANNING', 'No new target; checking subsequent map updates')
        if self.map_revision not in self.empty_revisions:
            self.empty_revisions.append(self.map_revision)
        if len(self.empty_revisions) >= 3:
            if self.failures:
                self._stop('EXHAUSTED', 'No further reachable untried targets; some motions failed')
            elif self.target_id >= 0:
                self._stop('NOT_FOUND', 'Target not seen from reachable sampled viewpoints; '
                           'hidden, blocked or inaccessible areas may remain')
            else:
                self._stop('COMPLETE', 'No further reachable untried frontier viewpoints; '
                           'inaccessible or unresolved unknown areas may remain')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ExplorationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Request cancellation and give callbacks a bounded opportunity to
        # acknowledge it while the ROS context remains available.
        if rclpy.ok() and node.active():
            node._stop('CANCELLED', 'Coordinator shutting down')
            until = time.monotonic() + 2.0
            while node.motion is not None and rclpy.ok() and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
