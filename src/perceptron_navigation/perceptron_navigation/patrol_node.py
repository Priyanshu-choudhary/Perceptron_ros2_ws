#!/usr/bin/env python3
"""High-level patrol: explore, find the dock, go to it, dock, and watch power.

    ros2 run perceptron_navigation patrol start
    ros2 run perceptron_navigation patrol status
    ros2 run perceptron_navigation patrol cancel

WHAT THIS IS
A supervisor. It owns no motion of its own: every movement is a Nav2 action or a
service call into the exploration and docking nodes, which already do those jobs
and are tested doing them. What lives here and nowhere else is the POLICY - what
to attempt next, and what power state should interrupt it.

    IDLE -> EXPLORING -- marker found --> GO_TO_DOCK -> DOCKING -> DOCKED
              |                               ^
              +-- nothing found --------+     |
                                        v     |
                                   RETURN_HOME
    battery low      -> dock known ? GO_TO_DOCK : RETURN_HOME
    battery critical -> cancel everything, STOPPED, no further motion

WHY A SEPARATE NODE
exploration_node is a good explorer and mission_node is a good fixed-route
patrol. Putting battery policy inside either would tie searching to power, and
tie power to a route. Here they stay independent and this node stays the only
thing that has to know the difference between "low" and "critical".

BATTERY POLICY
`low` is a deadline, not an emergency: finish approaching a known dock, or head
home. `critical` is an emergency: stop where you are. A dock approach that runs
out of charge halfway is the one case where stopping beats continuing, and a
robot parked somewhere inconvenient beats one that died mid-manoeuvre.

THE DOCK POSE IS ONLY AS GOOD AS THE MAP FRAME
The saved pose is in `map`. Under SLAM that frame is anchored to wherever the
robot started, so a pose saved in one run is meaningless in the next unless that
run relocalises against the same map. Within a run it is exact. See dock_store.
"""

import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from perceptron_navigation import dock_store

TERMINAL_EXPLORATION = ('FOUND', 'NOT_FOUND', 'COMPLETE', 'EXHAUSTED',
                        'FAILED', 'STOP_UNCONFIRMED')
RESTARTABLE = ('IDLE', 'DOCKED', 'HOME', 'STOPPED', 'FAILED')


class PatrolNode(Node):

    def __init__(self, **node_options):
        super().__init__('patrol_node', **node_options)
        self.declare_parameter('search_marker_id', 42)
        self.declare_parameter('dock_store_path', dock_store.DEFAULT_PATH)
        self.declare_parameter('dock_standoff', 1.20)
        self.declare_parameter('navigation_timeout', 300.0)
        self.declare_parameter('docking_timeout', 300.0)
        # Explore even when a dock is already on file. Off by default: if the
        # dock is the destination and it is already known, exploring first is
        # spending charge to rediscover something already recorded.
        self.declare_parameter('always_explore', False)

        self.dock_path = self.get_parameter('dock_store_path').value
        self.standoff = float(self.get_parameter('dock_standoff').value)
        self.marker_id = int(self.get_parameter('search_marker_id').value)

        self.state = 'IDLE'
        self.detail = 'Use patrol start'
        self.home = None
        self.dock = dock_store.load(self.dock_path)
        self.exploration_state = None
        self.docking_state = None
        self.low = False
        self.critical = False
        self.percentage = None
        self.goal_handle = None
        self.goal_kind = None
        self.goal_result = None
        self.deadline = 0.0
        self.interrupted = False

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        cb = ReentrantCallbackGroup()
        self.status_pub = self.create_publisher(String, '/patrol/status', 10)
        self.create_subscription(PoseStamped, '/exploration/found_marker',
                                 self._found, 10, callback_group=cb)
        self.create_subscription(String, '/exploration/status',
                                 self._exploration, 10, callback_group=cb)
        self.create_subscription(String, '/docking/status',
                                 self._docking, 10, callback_group=cb)
        self.create_subscription(Bool, '/battery/low', self._battery_low, 10,
                                 callback_group=cb)
        self.create_subscription(Bool, '/battery/critical', self._battery_critical,
                                 10, callback_group=cb)
        self.create_subscription(BatteryState, '/battery/state', self._battery,
                                 10, callback_group=cb)

        self.explore_start = self.create_client(Trigger, '/exploration/start',
                                                callback_group=cb)
        self.explore_cancel = self.create_client(Trigger, '/exploration/cancel',
                                                 callback_group=cb)
        self.dock_start = self.create_client(Trigger, '/docking/start', callback_group=cb)
        self.dock_cancel = self.create_client(Trigger, '/docking/cancel',
                                              callback_group=cb)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose',
                                callback_group=cb)

        self.create_service(Trigger, '/patrol/start', self._srv_start, callback_group=cb)
        self.create_service(Trigger, '/patrol/cancel', self._srv_cancel, callback_group=cb)
        self.create_timer(0.5, self._tick, callback_group=cb)

        known = 'a dock pose is already on file' if self.dock else 'no dock pose on file'
        self.get_logger().info(f'Patrol ready ({known}). Start it with:  '
                               'ros2 run perceptron_navigation patrol start')

    # ----------------------------------------------------------------- inputs

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _battery(self, msg):
        self.percentage = msg.percentage

    def _battery_low(self, msg):
        self.low = bool(msg.data)

    def _battery_critical(self, msg):
        self.critical = bool(msg.data)

    def _exploration(self, msg):
        try:
            self.exploration_state = json.loads(msg.data).get('state')
        except ValueError:
            self.exploration_state = None

    def _docking(self, msg):
        self.docking_state = msg.data

    def _found(self, msg):
        """Exploration confirmed the marker: persist it as the dock."""
        position = msg.pose.position
        orientation = msg.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        try:
            staging = dock_store.staging_pose(
                (position.x, position.y), quaternion, self.standoff)
        except ValueError as exc:
            self.get_logger().error(f'Cannot use this marker as a dock: {exc}')
            return
        self.dock = {
            'marker_id': self.marker_id,
            'frame': msg.header.frame_id or 'map',
            'position': {'x': position.x, 'y': position.y, 'z': position.z},
            'orientation': dict(zip(('x', 'y', 'z', 'w'), quaternion)),
            'staging': dict(zip(('x', 'y', 'yaw'), staging)),
            'stamp': self.now(),
        }
        path = dock_store.save(self.dock_path, self.dock)
        self.get_logger().info(
            f'Dock saved from marker {self.marker_id} at '
            f'({position.x:.2f}, {position.y:.2f}), staging '
            f'({staging[0]:.2f}, {staging[1]:.2f}) -> {path}')

    # --------------------------------------------------------------- services

    def _srv_start(self, request, response):
        if self.state not in RESTARTABLE:
            response.success = False
            response.message = f'Patrol already running ({self.state}).'
            return response
        if self.critical:
            response.success = False
            response.message = 'Battery critical; refusing to start.'
            return response
        self.home = None
        self.interrupted = False
        self.goal_handle = None
        self.goal_kind = None
        self.goal_result = None
        self._set('STARTING', 'Waiting for a robot pose to call home')
        response.success = True
        response.message = 'Patrol started.'
        return response

    def _srv_cancel(self, request, response):
        self._abort_activity()
        self._set('IDLE', 'Cancelled')
        response.success = True
        response.message = 'Patrol cancelled.'
        return response

    # ------------------------------------------------------------------ state

    def _set(self, state, detail):
        if (state, detail) != (self.state, self.detail):
            self.get_logger().info(f'{state}: {detail}')
        self.state, self.detail = state, detail
        self._publish()

    def _publish(self):
        self.status_pub.publish(String(data=json.dumps({
            'state': self.state,
            'detail': self.detail,
            'battery': self.percentage,
            'battery_low': self.low,
            'battery_critical': self.critical,
            'dock_known': self.dock is not None,
            'marker_id': self.marker_id,
        })))

    def _call(self, client, what):
        if not client.service_is_ready():
            self.get_logger().warn(f'{what} is not available',
                                   throttle_duration_sec=5.0)
            return False
        client.call_async(Trigger.Request())
        return True

    def _abort_activity(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.goal_kind = None
        self.goal_result = None
        if self.exploration_state is not None and \
                self.exploration_state not in TERMINAL_EXPLORATION:
            self._call(self.explore_cancel, '/exploration/cancel')
        if self.docking_state not in (None, 'IDLE', 'DOCKED', 'FAILED'):
            self._call(self.dock_cancel, '/docking/cancel')

    # ------------------------------------------------------------- navigation

    def _robot(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return (t.x, t.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                     1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def _navigate(self, x, y, yaw, kind):
        if not self.nav.server_is_ready():
            self.get_logger().warn('Nav2 action server is not ready',
                                   throttle_duration_sec=5.0)
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        # Stamp left at zero, meaning "use the latest transform". A fixed stamp
        # goes stale between replans and the goal is aborted for no visible
        # reason - the same trap the docking controller hit.
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_kind = kind
        self.goal_result = None
        self.deadline = self.now() + float(
            self.get_parameter('navigation_timeout').value)
        self.nav.send_goal_async(goal).add_done_callback(self._accepted)
        return True

    def _accepted(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.goal_result = 'failed'
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._result)

    def _result(self, future):
        try:
            self.goal_result = 'succeeded' if future.result().status == 4 else 'failed'
        except Exception:                                      # noqa: BLE001
            self.goal_result = 'failed'
        self.goal_handle = None

    # ------------------------------------------------------------------- tick

    def _tick(self):
        self._publish()
        if self.state in RESTARTABLE:
            return

        # Critical beats every other consideration, including a dock two metres
        # away. Whatever is moving, stop it.
        if self.critical:
            self._abort_activity()
            self._set('STOPPED', 'Battery critical; all motion stopped')
            return

        if self.state == 'STARTING':
            robot = self._robot()
            if robot is None:
                self.detail = 'Waiting for map -> base_footprint'
                return
            self.home = robot
            if self.dock is not None and not self.get_parameter('always_explore').value:
                self._set('GO_TO_DOCK', 'Dock already on file; skipping exploration')
            elif self._call(self.explore_start, '/exploration/start'):
                self._set('EXPLORING',
                          f'Mapping and searching for marker {self.marker_id}')
            return

        if self.low and not self.interrupted and self.state == 'EXPLORING':
            # A deadline, not an emergency: stop searching and spend what charge
            # is left getting somewhere useful.
            self.interrupted = True
            self._abort_activity()
            if self.dock is not None:
                self._set('GO_TO_DOCK', 'Battery low; heading for the known dock')
            else:
                self._set('RETURN_HOME', 'Battery low and no dock found; heading home')
            return

        handler = getattr(self, '_run_' + self.state.lower(), None)
        if handler is not None:
            handler()

    def _run_exploring(self):
        if self.dock is not None:
            self._abort_activity()
            self._set('GO_TO_DOCK', 'Marker found; driving to the dock')
            return
        if self.exploration_state in TERMINAL_EXPLORATION:
            self._set('RETURN_HOME',
                      f'Exploration ended ({self.exploration_state}) without the marker')

    def _run_go_to_dock(self):
        if self.goal_kind != 'dock':
            staging = self.dock.get('staging')
            if not staging:
                self._set('RETURN_HOME', 'Saved dock has no staging pose')
                return
            if self._navigate(staging['x'], staging['y'], staging['yaw'], 'dock'):
                self.detail = ('Driving to the dock staging pose '
                               f"({staging['x']:.2f}, {staging['y']:.2f})")
            return
        if self.goal_result == 'succeeded':
            # Keep goal_kind set. Clearing it here meant that a /docking/start
            # which was not yet available sent the supervisor back around to
            # re-navigate to a staging pose it was already standing on - 555
            # navigation goals in one measured run, and never a handover.
            if self._call(self.dock_start, '/docking/start'):
                self.goal_kind = None
                self.deadline = self.now() + float(
                    self.get_parameter('docking_timeout').value)
                self._set('DOCKING', 'At the staging pose; handing over to ArUco docking')
            elif self.now() > self.deadline:
                self.goal_kind = None
                self._set('RETURN_HOME',
                          'Docking controller never answered; heading home')
            else:
                self.detail = 'At the staging pose; waiting for the docking controller'
            return
        if self.goal_result == 'failed' or self.now() > self.deadline:
            self.goal_kind = None
            self._set('RETURN_HOME', 'Could not reach the dock; heading home')

    def _run_docking(self):
        if self.docking_state == 'DOCKED':
            self._set('DOCKED', 'Docked')
            return
        if self.docking_state == 'FAILED' or self.now() > self.deadline:
            self._call(self.dock_cancel, '/docking/cancel')
            self._set('RETURN_HOME', 'Docking did not succeed; heading home')

    def _run_return_home(self):
        if self.home is None:
            self._set('FAILED', 'No home pose was recorded')
            return
        if self.goal_kind != 'home':
            if self._navigate(self.home[0], self.home[1], self.home[2], 'home'):
                self.detail = (f'Returning to ({self.home[0]:.2f}, '
                               f'{self.home[1]:.2f})')
            return
        if self.goal_result == 'succeeded':
            self._set('HOME', 'Back at the start position')
        elif self.goal_result == 'failed' or self.now() > self.deadline:
            self._set('FAILED', 'Could not get home')


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
