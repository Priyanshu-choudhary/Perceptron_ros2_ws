#!/usr/bin/env python3
"""Closed-loop ArUco docking controller.

Services
    /docking/start   (std_srvs/Trigger)  begin docking
    /docking/cancel  (std_srvs/Trigger)  abort and stop

Topics
    /docking/status        (std_msgs/String)      current state, 20 Hz
    /docking/marker_pose   (PoseStamped, in)      from aruco_detector_node
    /docking/dock_estimate (PoseStamped, out)     the tracked dock pose, in odom
    /docking/predock_pose  (PoseStamped, out)     the waypoint being driven to

State machine
    IDLE --start--> SEARCHING --marker--> APPROACH --near--> ALIGNING
              |         ^                 (Nav2 drives)            |
              |         +---- LOST_RECOVERY ------------+   FINAL_APPROACH
              +-- saved dock on file -----^                        |
                                                                   |
                                                            FINAL_CONTACT
                                                                   |
                                                                 DOCKED

APPROACH is Nav2 covering the distance with obstacle avoidance. ALIGNING is the
last half metre onto the pre-dock pose, done here because it is a shuffle
sideways onto the dock axis, and that is the one manoeuvre a differential base
does well and a trajectory controller does badly.

Two ideas do most of the work here.

1. The dock is tracked in the ODOM frame, not the camera frame. Every detection
   is converted to odom and blended into a running estimate; the control loop
   converts that estimate back into robot coordinates each tick. Because the
   dock is static, filtering in odom is free smoothing with no lag, and - more
   importantly - the robot can turn away from the marker mid-manoeuvre without
   losing its target. Tracking only what the camera can see right now makes the
   robot oscillate between APPROACH and SEARCHING forever on any approach that
   is not already head-on.

2. APPROACH drives to a PRE-DOCK POSE, not to the marker: a point
   `predock_standoff` metres out along the marker's own normal, facing the
   marker. Homing straight at the marker arrives at the dock at an angle and
   wedges the robot against the plate. Going via the normal guarantees a square
   final approach from any starting position.

   That pre-dock pose is now handed to Nav2 as a NavigateToPose goal rather than
   driven by a local pose controller. The old controller flew a direct
   rho/alpha/beta path with no idea what was in the way, so anything parked
   between the robot and the dock ended the attempt. Nav2 plans around it and
   brings its own recovery behaviours. The goal is published in the ODOM frame,
   which is where the dock estimate already lives, so this works whether map is
   supplied by AMCL, by slam_toolbox, or not at all.

   While Nav2 drives, refined detections keep moving the pre-dock pose. If it
   moves more than `predock_goal_update_threshold`, the goal is re-sent, so the
   approach converges on the marker rather than on the first guess at it.

WHO DRIVES THE BASE
Only one thing may command the wheels at a time, and cmd_vel_relay is the
switch: Nav2's output reaches the base only while the relay is enabled. This
node owns that switch for the whole docking sequence - relay ON for the Nav2
approach, OFF for the visual final approach and the blind push, because those
are driven from here. The mission node no longer touches it.

A dock recorded by a previous patrol (dock_store_path) skips SEARCHING: there
is nothing to search for when the answer is on file, so the estimate is seeded
from it and the approach starts immediately. The camera still has to confirm the
marker before the final approach, and if it never does the estimate goes stale
and the robot falls back to sweeping. Set use_saved_dock:=false to always search.

FINAL_CONTACT is open loop on odometry. Close in, the 10 cm marker overflows the
camera's field of view and detection stops - the same thing happens on the real
robot - so the last few centimetres are dead reckoned.
"""

import json
import math
import os
from enum import Enum
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener


class DockingState(Enum):
    IDLE = 'IDLE'
    SEARCHING = 'SEARCHING'
    APPROACH = 'APPROACH'
    ALIGNING = 'ALIGNING'
    FINAL_APPROACH = 'FINAL_APPROACH'
    FINAL_CONTACT = 'FINAL_CONTACT'
    DOCKED = 'DOCKED'
    LOST_RECOVERY = 'LOST_RECOVERY'
    FAILED = 'FAILED'


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def marker_normal_yaw(q) -> float:
    """Planar heading of the marker's +Z axis, which points out of its face.

    Returns None when the marker is seen so obliquely that the normal barely
    projects onto the ground plane and its direction is mostly noise.
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    nx = 2.0 * (x * z + w * y)
    ny = 2.0 * (y * z - w * x)
    if math.hypot(nx, ny) < 0.15:
        return None
    return math.atan2(ny, nx)


class DockingControllerNode(Node):

    def __init__(self):
        super().__init__('docking_controller_node')

        # --- geometry of the docking manoeuvre ---
        self.declare_parameter('target_docking_distance', 0.35)
        self.declare_parameter('predock_standoff', 0.75)
        self.declare_parameter('final_push_distance', 0.14)
        self.declare_parameter('contact_timeout_seconds', 8.0)
        self.declare_parameter('predock_position_tolerance', 0.06)
        self.declare_parameter('distance_tolerance', 0.02)
        self.declare_parameter('angle_tolerance', 0.05)
        self.declare_parameter('lateral_tolerance', 0.02)

        # --- velocity envelope ---
        self.declare_parameter('max_linear_velocity', 0.25)
        self.declare_parameter('min_linear_velocity', 0.04)
        self.declare_parameter('max_angular_velocity', 0.60)
        self.declare_parameter('search_angular_velocity', 0.35)
        self.declare_parameter('final_linear_velocity', 0.07)
        self.declare_parameter('contact_linear_velocity', 0.05)

        # --- gains ---
        # Nav2 drives the approach leg; these are the handover knobs.
        self.declare_parameter('nav2_action', '/navigate_to_pose')
        # Nav2 needs its whole lifecycle up before it will answer, which from a
        # cold launch is comfortably longer than the marker search takes.
        self.declare_parameter('nav2_wait_seconds', 45.0)
        self.declare_parameter('predock_goal_update_threshold', 0.20)
        # Empty means "whatever bt_navigator was launched with", which uses the
        # 0.15 m / 0.20 rad default goal checker - too coarse to hand over to a
        # visual servo. The shipped tree asks for precise_goal_checker instead.
        self.declare_parameter('nav2_behavior_tree', '')
        # Final placement, once Nav2 has delivered the robot near the pre-dock
        # pose. Bounded: beyond align_max_range this hands back to Nav2.
        self.declare_parameter('align_max_range', 0.80)
        self.declare_parameter('nav2_max_attempts', 3)
        self.declare_parameter('max_align_handbacks', 3)
        # A dock recorded by a previous patrol. With one on file there is
        # nothing to search FOR: the approach can start immediately and the
        # camera only has to confirm it on the way in.
        self.declare_parameter('use_saved_dock', True)
        self.declare_parameter('dock_store_path', '~/.ros/perceptron_dock/dock.json')
        self.declare_parameter('align_k_rho', 0.55)
        self.declare_parameter('align_k_alpha', 1.40)
        self.declare_parameter('align_k_beta', -0.55)
        self.declare_parameter('relay_enable_service', '/cmd_vel_relay/enable')
        self.declare_parameter('kp_final_heading', 1.2)
        self.declare_parameter('kp_final_lateral', 1.6)
        self.declare_parameter('pose_filter_alpha', 0.3)

        # --- watchdogs / io ---
        self.declare_parameter('marker_timeout_seconds', 1.5)
        self.declare_parameter('estimate_timeout_seconds', 25.0)
        self.declare_parameter('max_docking_time_seconds', 180.0)
        self.declare_parameter('cmd_vel_topic', '/diff_drive_controller/cmd_vel_unstamped')
        self.declare_parameter('odom_topic', '/diff_drive_controller/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('auto_start', False, ParameterDescriptor(dynamic_typing=True))

        g = self._param
        self.d_target = g('target_docking_distance')
        self.standoff = g('predock_standoff')
        self.push_distance = g('final_push_distance')
        self.contact_timeout = g('contact_timeout_seconds')
        self.predock_tol = g('predock_position_tolerance')
        self.tol_dist = g('distance_tolerance')
        self.tol_angle = g('angle_tolerance')
        self.tol_lat = g('lateral_tolerance')

        self.v_max = g('max_linear_velocity')
        self.v_min = g('min_linear_velocity')
        self.w_max = g('max_angular_velocity')
        self.w_search = g('search_angular_velocity')
        self.v_final = g('final_linear_velocity')
        self.v_contact = g('contact_linear_velocity')

        self.nav2_wait = g('nav2_wait_seconds')
        self.goal_update_threshold = g('predock_goal_update_threshold')
        self.align_max_range = g('align_max_range')
        self.nav2_max_attempts = int(
            self.get_parameter('nav2_max_attempts').value)
        self.max_handbacks = int(self.get_parameter('max_align_handbacks').value)
        self.use_saved_dock = bool(self.get_parameter('use_saved_dock').value)
        self.dock_store_path = self.get_parameter('dock_store_path').value
        self.k_rho = g('align_k_rho')
        self.k_alpha = g('align_k_alpha')
        self.k_beta = g('align_k_beta')
        self.nav2_bt = self.get_parameter('nav2_behavior_tree').value
        if not self.nav2_bt:
            from ament_index_python.packages import get_package_share_directory
            self.nav2_bt = os.path.join(
                get_package_share_directory('perceptron_docking'),
                'behavior_trees', 'dock_approach.xml')
        self.kp_head = g('kp_final_heading')
        self.kp_lat = g('kp_final_lateral')
        self.filter_alpha = g('pose_filter_alpha')

        self.timeout_sec = g('marker_timeout_seconds')
        self.estimate_timeout = g('estimate_timeout_seconds')
        self.max_time = g('max_docking_time_seconds')
        self.odom_frame = self.get_parameter('odom_frame').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value

        auto = self.get_parameter('auto_start').value
        auto_start = auto if isinstance(auto, bool) else \
            str(auto).strip().lower() in ('true', '1', 'yes')

        # --- state ---
        self.state = DockingState.IDLE
        self.odom = None          # (x, y, yaw) of base_footprint in odom
        self.dock_odom = None     # (x, y, normal_yaw) of the marker in odom
        self.last_seen = None     # node-clock time of the last accepted detection
        self.start_time = None
        self.search_direction = 1.0
        self.contact_origin = None
        self.contact_start = None
        self._backing_off = False
        self._raw_base = None     # fallback when there is no odometry at all
        self._nav_goal_handle = None   # live NavigateToPose goal, if any
        self._nav_result = None        # 'running' | 'succeeded' | 'failed'
        self._nav_goal_odom = None     # the (x, y, yaw) the goal was sent for
        self._nav_sent_at = None
        # Re-targeting cancels the live goal and sends a new one. The cancelled
        # goal's result callback arrives AFTER the replacement is already
        # running, and it reports CANCELED, so without a generation tag it marks
        # the new goal failed and aborts a perfectly healthy approach.
        self._nav_generation = 0
        self._pending_target = None
        self._align_handbacks = 0
        self._nav_attempts = 0
        self._relay_enabled = None     # last value pushed to cmd_vel_relay

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.status_pub = self.create_publisher(String, '/docking/status', 10)
        self.predock_pub = self.create_publisher(PoseStamped, '/docking/predock_pose', 10)
        self.estimate_pub = self.create_publisher(PoseStamped, '/docking/dock_estimate', 10)
        self.create_subscription(PoseStamped, '/docking/marker_pose', self._marker_cb, 10)
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                 self._odom_cb, 10)

        cb = ReentrantCallbackGroup()
        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.nav_client = ActionClient(
            self, NavigateToPose, self.get_parameter('nav2_action').value,
            callback_group=cb)
        self.relay_client = self.create_client(
            SetBool, self.get_parameter('relay_enable_service').value,
            callback_group=cb)

        self.create_service(Trigger, '/docking/start', self._srv_start,
                            callback_group=cb)
        self.create_service(Trigger, '/docking/cancel', self._srv_cancel,
                            callback_group=cb)

        self.timer = self.create_timer(0.05, self._control_loop, callback_group=cb)

        self.get_logger().info(
            f'Docking controller ready, commanding {cmd_topic}. '
            f'Pre-dock standoff {self.standoff:.2f} m, visual stop {self.d_target:.2f} m, '
            f'then a {self.push_distance:.2f} m dead-reckoned push.')
        if auto_start:
            self._begin('auto_start')
        else:
            self.get_logger().info(
                'Idle. Start it with:  ros2 run perceptron_docking dock start')

    def _param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    # ---------------------------------------------------------------- inputs

    def _now(self) -> float:
        """Seconds from the node clock, so this honours use_sim_time."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_cb(self, msg: Odometry):
        self.odom = (msg.pose.pose.position.x,
                     msg.pose.pose.position.y,
                     yaw_of(msg.pose.pose.orientation))

    def _marker_cb(self, msg: PoseStamped):
        mx, my = msg.pose.position.x, msg.pose.position.y
        nyaw = marker_normal_yaw(msg.pose.orientation)
        if nyaw is None:
            # Edge-on view: assume the dock faces us rather than trusting noise.
            nyaw = normalize_angle(math.atan2(my, mx) + math.pi)

        self._raw_base = (mx, my, nyaw)
        self.last_seen = self._now()

        if self.odom is None:
            return

        ox, oy, oyaw = self.odom
        c, s = math.cos(oyaw), math.sin(oyaw)
        obs = (ox + mx * c - my * s,
               oy + mx * s + my * c,
               normalize_angle(oyaw + nyaw))

        if self.dock_odom is None:
            self.dock_odom = obs
        else:
            a = self.filter_alpha
            px, py, pyaw = self.dock_odom
            # The dock does not move, so blending in odom is pure noise
            # rejection - unlike filtering in the robot frame, it adds no lag.
            self.dock_odom = (px + a * (obs[0] - px),
                              py + a * (obs[1] - py),
                              normalize_angle(pyaw + a * normalize_angle(obs[2] - pyaw)))

    # --------------------------------------------------------------- services

    def _srv_start(self, request, response):
        if self.state in (DockingState.SEARCHING, DockingState.APPROACH,
                          DockingState.ALIGNING, DockingState.FINAL_APPROACH,
                          DockingState.FINAL_CONTACT):
            response.success = False
            response.message = f'Already docking (state {self.state.value}).'
            return response
        self._begin('service call')
        response.success = True
        response.message = 'Docking sequence started.'
        return response

    def _srv_cancel(self, request, response):
        self._cancel_nav_goal()
        self._stop()
        self.state = DockingState.IDLE
        self.start_time = None
        self.contact_origin = None
        # Whoever called us is in charge of the base again.
        self._set_relay(True)
        self.get_logger().warn('Docking cancelled.')
        response.success = True
        response.message = 'Docking aborted.'
        return response

    def _saved_dock_in_odom(self):
        """The dock recorded by a previous patrol, expressed in odom, or None.

        The file is written by perceptron_navigation's patrol supervisor in the
        MAP frame. This node tracks the dock in odom, so the pose has to be
        carried across; that also means the answer is only as good as the
        map -> odom transform currently is, which is the honest constraint
        either way.

        Read directly rather than importing the writer's module:
        perceptron_navigation already exec_depends on this package, and a
        dependency back would be a cycle colcon refuses to order.
        """
        if not self.use_saved_dock:
            return None
        try:
            record = json.loads(
                Path(self.dock_store_path).expanduser().read_text(encoding='utf-8'))
            position = record['position']
            orientation = record['orientation']
            frame = record.get('frame', 'map')
            px, py = float(position['x']), float(position['y'])
            qx, qy = float(orientation['x']), float(orientation['y'])
            qz, qw = float(orientation['z']), float(orientation['w'])
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.get_logger().info(f'No usable saved dock ({exc}); searching instead.')
            return None

        # The marker's own +Z is its face normal, flattened into the floor plane.
        nx = 2.0 * (qx * qz + qw * qy)
        ny = 2.0 * (qy * qz - qw * qx)
        if math.hypot(nx, ny) < 1e-6:
            self.get_logger().warn('Saved dock normal is vertical; searching instead.')
            return None
        normal_yaw = math.atan2(ny, nx)

        try:
            tf = self.tf_buffer.lookup_transform(self.odom_frame, frame, Time())
        except TransformException as exc:
            self.get_logger().warn(
                f'Saved dock is in {frame} but {frame} -> {self.odom_frame} is not '
                f'available ({exc}); searching instead.')
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        tf_yaw = math.atan2(2.0 * (r.w * r.z + r.x * r.y),
                            1.0 - 2.0 * (r.y * r.y + r.z * r.z))
        c, sn = math.cos(tf_yaw), math.sin(tf_yaw)
        return (t.x + c * px - sn * py,
                t.y + sn * px + c * py,
                normalize_angle(normal_yaw + tf_yaw))

    def _begin(self, source: str):
        self.state = DockingState.SEARCHING
        self.start_time = self._now()
        self.last_seen = None
        self.dock_odom = None
        self._raw_base = None
        self.contact_origin = None
        self._backing_off = False
        self._nav_goal_handle = None
        self._nav_result = None
        self._nav_goal_odom = None
        self._nav_sent_at = None
        self._nav_attempts = 0
        self._align_handbacks = 0
        # Searching spins on the spot from here, so take the base now.
        self._set_relay(False)

        saved = self._saved_dock_in_odom()
        if saved is not None:
            # Skip SEARCHING entirely. The estimate is seeded from the file and
            # last_seen is set so the approach may start; if the camera never
            # confirms it on the way in, the estimate goes stale, LOST_RECOVERY
            # fires and the robot falls back to sweeping for it. A saved pose is
            # a strong hint, not a promise.
            self.dock_odom = saved
            self.last_seen = self._now()
            self.state = DockingState.APPROACH
            self.get_logger().info(
                f'Docking started ({source}). Using the saved dock at '
                f'({saved[0]:.2f}, {saved[1]:.2f}) in {self.odom_frame}; '
                'skipping the search.')
            return
        self.get_logger().info(f'Docking started ({source}). Searching for the marker.')

    # ------------------------------------------------------------------ utils

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _drive(self, v: float, w: float):
        cmd = Twist()
        cmd.linear.x = clamp(v, -self.v_max, self.v_max)
        cmd.angular.z = clamp(w, -self.w_max, self.w_max)
        self.cmd_pub.publish(cmd)

    def _visual_fresh(self) -> bool:
        return self.last_seen is not None and (self._now() - self.last_seen) < self.timeout_sec

    def _dock_in_base(self):
        """Marker pose as (x, y, normal_yaw) in base_footprint, right now.

        Derived from the odom-frame estimate, so it stays valid while the
        marker is out of view. Falls back to the raw detection if there is no
        odometry to propagate with.
        """
        if self.dock_odom is not None and self.odom is not None:
            ox, oy, oyaw = self.odom
            dx, dy, dyaw = self.dock_odom
            c, s = math.cos(oyaw), math.sin(oyaw)
            rx, ry = dx - ox, dy - oy
            return (rx * c + ry * s,
                    -rx * s + ry * c,
                    normalize_angle(dyaw - oyaw))
        if self._visual_fresh():
            return self._raw_base
        return None

    def _estimate_valid(self) -> bool:
        return (self._dock_in_base() is not None
                and self.last_seen is not None
                and (self._now() - self.last_seen) < self.estimate_timeout)

    def _predock_pose(self, marker):
        """Pre-dock waypoint (x, y, heading) in base_footprint."""
        mx, my, nyaw = marker
        return (mx + self.standoff * math.cos(nyaw),
                my + self.standoff * math.sin(nyaw),
                normalize_angle(nyaw + math.pi))

    def _publish_debug(self, marker):
        px, py, heading = self._predock_pose(marker)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.pose.position.x = px
        msg.pose.position.y = py
        msg.pose.orientation.z = math.sin(heading / 2.0)
        msg.pose.orientation.w = math.cos(heading / 2.0)
        self.predock_pub.publish(msg)

        if self.dock_odom is not None:
            dx, dy, dyaw = self.dock_odom
            est = PoseStamped()
            est.header.stamp = msg.header.stamp
            est.header.frame_id = self.odom_frame
            est.pose.position.x = dx
            est.pose.position.y = dy
            est.pose.orientation.z = math.sin(dyaw / 2.0)
            est.pose.orientation.w = math.cos(dyaw / 2.0)
            self.estimate_pub.publish(est)

    def _fail(self, reason: str):
        self._cancel_nav_goal()
        self._stop()
        self.state = DockingState.FAILED
        self._set_relay(True)
        self.get_logger().error(f'Docking failed: {reason}')

    # ------------------------------------------------------------ control loop

    def _control_loop(self):
        self.status_pub.publish(String(data=self.state.value))

        if self.state in (DockingState.IDLE, DockingState.DOCKED, DockingState.FAILED):
            return

        if self.start_time is not None and (self._now() - self.start_time) > self.max_time:
            self._fail('took longer than max_docking_time_seconds')
            return

        # The last few centimetres are deliberately blind; do not let the
        # marker watchdog abort them.
        if self.state == DockingState.FINAL_CONTACT:
            self._set_relay(False)
            self._run_final_contact()
            return

        marker = self._dock_in_base()

        # APPROACH is handled before the freshness checks below. Nav2 may drive
        # for well over estimate_timeout_seconds with the marker out of frame -
        # it is behind the robot for most of an off-axis approach - and the goal
        # is already pinned in odom, so losing sight of it is not a failure here.
        if self.state == DockingState.APPROACH:
            if marker is None:
                self._fail('no dock estimate to approach')
            else:
                self._publish_debug(marker)
                self._run_approach(marker)
            return

        if self.state == DockingState.SEARCHING:
            if marker is not None and self._visual_fresh():
                self.get_logger().info('Marker acquired, driving to the pre-dock pose.')
                self.state = DockingState.APPROACH
                self._stop()
            else:
                self._drive(0.0, self.w_search * self.search_direction)
            return

        if self.state == DockingState.LOST_RECOVERY:
            self._stop()
            if self._visual_fresh():
                self.get_logger().info('Marker re-acquired.')
                self.state = DockingState.APPROACH
            elif self.last_seen is None or (self._now() - self.last_seen) > self.timeout_sec * 4.0:
                self.get_logger().warn('Marker still lost, sweeping again.')
                self.dock_odom = None
                self._raw_base = None
                self.last_seen = None
                self.state = DockingState.SEARCHING
            return

        if not self._estimate_valid():
            self.get_logger().warn('Lost track of the dock.')
            self._stop()
            self.state = DockingState.LOST_RECOVERY
            return

        self._publish_debug(marker)

        if self.state == DockingState.ALIGNING:
            self._set_relay(False)
            self._run_align(marker)
        elif self.state == DockingState.FINAL_APPROACH:
            self._set_relay(False)
            self._run_final_approach(marker)

    # ------------------------------------------------------- Nav2 approach

    def _set_relay(self, enabled: bool):
        """Hand the base to Nav2 (True) or take it back (False).

        Idempotent: only actually calls the service when the value changes, so
        this is safe to call every tick from the control loop.
        """
        if self._relay_enabled == enabled:
            return
        if not self.relay_client.service_is_ready():
            if self._relay_enabled is None:
                self.get_logger().warn(
                    'cmd_vel_relay is not up; running docking without arbitration. '
                    'If Nav2 is also driving, the two will fight over the base.')
                self._relay_enabled = enabled
            return
        req = SetBool.Request()
        req.data = enabled
        self.relay_client.call_async(req)
        self._relay_enabled = enabled
        self.get_logger().info(f'cmd_vel_relay {"enabled, Nav2 drives" if enabled else "disabled, docking drives"}.')

    def _predock_in_odom(self, marker):
        """The pre-dock waypoint as (x, y, yaw) in the odom frame."""
        px, py, heading = self._predock_pose(marker)
        if self.odom is None:
            return None
        ox, oy, oyaw = self.odom
        c, s_ = math.cos(oyaw), math.sin(oyaw)
        return (ox + px * c - py * s_,
                oy + px * s_ + py * c,
                normalize_angle(heading + oyaw))

    def _send_nav_goal(self, target):
        x, y, yaw = target
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.odom_frame
        # Stamp zero, meaning "use the latest transform available", NOT now().
        # The planner re-transforms this pose on every replan, and a fixed
        # timestamp goes stale within seconds: tf drops it out of the buffer and
        # every replan fails with "Lookup would require extrapolation into the
        # past", which surfaces as the goal being aborted for no visible reason.
        # The dock does not move, so there is no wrong answer to pick here.
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal.behavior_tree = self.nav2_bt

        self._nav_generation += 1
        gen = self._nav_generation
        self._nav_result = 'running'
        self._nav_goal_odom = target
        self._nav_sent_at = self._now()
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._on_goal_response(f, gen))
        self.get_logger().info(
            f'Nav2 goal: pre-dock pose ({x:+.2f}, {y:+.2f}) in {self.odom_frame}.')

    def _on_goal_response(self, future, gen):
        if gen != self._nav_generation:
            return                      # superseded by a re-target
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Nav2 rejected the pre-dock goal.')
            self._nav_result = 'failed'
            return
        self._nav_goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: self._on_goal_result(f, gen))

    def _on_goal_result(self, future, gen):
        if gen != self._nav_generation:
            return                      # result of a goal we already replaced
        # 4 == STATUS_SUCCEEDED; anything else is an abort or a rejection.
        status = future.result().status
        self._nav_result = 'succeeded' if status == 4 else 'failed'
        self._nav_goal_handle = None

    def _retarget(self, target):
        """Swap the live goal for a new one, cancel first and goal second."""
        self._pending_target = target
        self._nav_result = 'cancelling'
        handle = self._nav_goal_handle
        self._nav_generation += 1
        self._nav_goal_handle = None
        if handle is None:
            self._after_cancel()
            return
        handle.cancel_goal_async().add_done_callback(lambda f: self._after_cancel())

    def _after_cancel(self):
        target, self._pending_target = self._pending_target, None
        if target is None or self.state != DockingState.APPROACH:
            return
        self._send_nav_goal(target)

    def _cancel_nav_goal(self):
        # Bump the generation here too, so a cancel that is NOT followed by a
        # new goal still orphans the outstanding callbacks.
        self._nav_generation += 1
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None
        self._nav_result = None

    def _run_approach(self, marker):
        """Nav2 drives to the pre-dock pose; we only re-target and hand over."""
        if self._nav_result is None:
            if not self.nav_client.server_is_ready():
                if self._nav_sent_at is None:
                    self._nav_sent_at = self._now()
                    self.get_logger().info('Waiting for the Nav2 action server...')
                if (self._now() - self._nav_sent_at) > self.nav2_wait:
                    self._fail(f'no Nav2 action server after {self.nav2_wait:.0f} s; '
                               'docking needs Nav2 to drive the approach')
                return
            target = self._predock_in_odom(marker)
            if target is None:
                return
            self._set_relay(True)
            self._send_nav_goal(target)
            return

        if self._nav_result == 'running':
            self._set_relay(True)
            # Detections keep refining the marker pose. Re-aim if the waypoint
            # has moved far enough to be worth the replan.
            target = self._predock_in_odom(marker)
            settled = (self._nav_sent_at is not None
                       and (self._now() - self._nav_sent_at) > 3.0)
            if (settled and target is not None and self._nav_goal_odom is not None
                    and self._visual_fresh()):
                moved = math.hypot(target[0] - self._nav_goal_odom[0],
                                   target[1] - self._nav_goal_odom[1])
                if moved > self.goal_update_threshold:
                    self.get_logger().info(
                        f'Pre-dock pose moved {moved:.2f} m; re-sending the goal.')
                    self._retarget(target)
            return

        if self._nav_result == 'cancelling':
            # Waiting for the old goal to actually go away. Sending the
            # replacement now would reach bt_navigator as a PREEMPTION of the
            # live goal, and a preemption that names a different behaviour tree
            # is rejected outright - which fails the whole approach.
            self._set_relay(True)
            return

        if self._nav_result == 'failed':
            # A failed navigation is not a failed dock. The usual causes are
            # transient - the global costmap dropping scans while tf catches up,
            # a replan landing on a momentarily lethal cell - and the marker
            # estimate is still perfectly good, so the honest response is to ask
            # again rather than abandon the attempt.
            target = self._predock_in_odom(marker)
            rho = math.hypot(*self._predock_pose(marker)[:2])
            if rho <= self.align_max_range:
                # Close enough that Nav2 has nothing left to contribute: the
                # remaining error is a placement problem, which ALIGNING owns.
                self.get_logger().warn(
                    f'Nav2 gave up {rho:.2f} m from the pre-dock pose; close '
                    'enough to finish locally.')
                self.state = DockingState.ALIGNING
                return
            self._nav_attempts += 1
            if self._nav_attempts >= self.nav2_max_attempts:
                self._fail(f'Nav2 could not reach the pre-dock pose after '
                           f'{self._nav_attempts} attempts')
                return
            self.get_logger().warn(
                f'Nav2 failed, retrying ({self._nav_attempts} of '
                f'{self.nav2_max_attempts}).')
            self._nav_result = None
            self._nav_sent_at = None
            return

        # Arrived, near enough. Take the base back for the final placement.
        self._set_relay(False)
        self._stop()
        self.get_logger().info('Nav2 delivered the robot; aligning on the '
                               'pre-dock pose.')
        self.state = DockingState.ALIGNING

    def _run_align(self, marker):
        """Put the robot exactly on the pre-dock pose, over the last half metre.

        This is NOT the old approach controller doing the whole journey again.
        It is bounded to align_max_range and it exists because of a specific
        thing Nav2 cannot do on a differential base: sit facing the dock a
        third of a metre to one side of the approach axis and shuffle sideways
        onto it. Reaching that pose means turning away from the goal, driving,
        and turning back, and every goal-angle critic in the controller is
        pushing against exactly that. MPPI gave up there with "Failed to make
        progress" from both off-axis starts, a third of a metre out.

        Turning on the spot and driving a short straight line is what this base
        is good at, so the last half metre is done here and the obstacle
        avoidance that justified Nav2 has already happened by now.
        """
        px, py, heading = self._predock_pose(marker)
        rho = math.hypot(px, py)

        if rho > self.align_max_range and self._align_handbacks < self.max_handbacks:
            # Too far to be a placement problem any more - something moved, or
            # the estimate was refined a long way. Give it back to Nav2.
            #
            # Bounded, because the two states can chase each other forever:
            # Nav2 arrives at the pre-dock pose, a fresh detection moves that
            # pose beyond align_max_range, ALIGNING hands back, Nav2 drives to
            # the new one, and so on. Measured from a saved dock 0.34 m off the
            # truth, that ping-pong ran until the docking timeout. After a few
            # attempts the local controller is simply the better tool: it is a
            # pose controller, and driving it a metre or two is slower than Nav2
            # but perfectly correct.
            self._align_handbacks += 1
            self.get_logger().warn(
                f'Pre-dock pose is {rho:.2f} m away; handing back to Nav2 '
                f'({self._align_handbacks} of {self.max_handbacks}).')
            self._nav_result = None
            self._nav_sent_at = None
            self.state = DockingState.APPROACH
            return

        heading_err = normalize_angle(marker[2] + math.pi)

        if rho < self.predock_tol:
            # On the waypoint. Turn on the spot: rho/alpha/beta is useless at
            # rho ~ 0, where alpha is the atan2 of two noisy near-zero numbers
            # and its output thrashes.
            if abs(heading_err) <= self.tol_angle * 3.0 and self._visual_fresh():
                self.get_logger().info(
                    f'At the pre-dock pose, marker {math.hypot(marker[0], marker[1]):.2f} m '
                    'ahead. Starting the final approach.')
                self.state = DockingState.FINAL_APPROACH
                self._backing_off = False
                self._stop()
                return
            turn = self.kp_head * heading_err
            if abs(turn) < 0.08:
                turn = math.copysign(0.08, turn) if abs(heading_err) > 1e-3 else 0.0
            self._drive(0.0, clamp(turn, -self.w_max * 0.6, self.w_max * 0.6))
            return

        alpha = normalize_angle(math.atan2(py, px))
        beta = normalize_angle(heading - alpha)

        # Waypoint beside or behind us: spin to face it rather than reversing
        # into the dock.
        if abs(alpha) > math.radians(75.0):
            self._drive(0.0, math.copysign(self.w_max * 0.7, alpha))
            return

        v = self.k_rho * rho
        w = self.k_alpha * alpha + self.k_beta * beta
        # Ease off in tight turns so the skid-steer does not scrub sideways.
        v *= max(0.25, 1.0 - abs(w) / self.w_max)
        self._drive(clamp(v, self.v_min, self.v_max), w)

    def _run_final_approach(self, marker):
        if not self._visual_fresh():
            # Precision work needs live vision. Drop back to ALIGNING, which
            # will re-point at the dock instead of blindly creeping forward.
            # Not APPROACH: the robot is already where Nav2 was asked to put it,
            # and a whole new navigation goal for half a metre is wasteful.
            self.get_logger().warn('Marker not in view during the final approach; backing off.')
            self.state = DockingState.ALIGNING
            self._stop()
            return

        mx, my, nyaw = marker
        c, sn = math.cos(nyaw), math.sin(nyaw)
        # Work in dock-axis coordinates rather than straight-line range:
        #   d_axis  distance to the marker measured along its own normal
        #   e_ct    cross-track error, positive when the dock axis is to our left
        # A differential drive cannot remove a lateral offset by turning on the
        # spot, so the cross-track has to be steered out while driving. This is
        # line following onto the dock axis, not point homing.
        d_axis = -(mx * c + my * sn)
        e_ct = mx * sn - my * c
        e_head = normalize_angle(nyaw + math.pi)
        e_d = d_axis - self.d_target

        if abs(e_d) <= self.tol_dist and abs(e_ct) <= self.tol_lat                 and abs(e_head) <= self.tol_angle:
            self.get_logger().info(
                f'Aligned at {d_axis:.3f} m on the dock axis (cross-track '
                f'{e_ct * 100:+.1f} cm, heading {math.degrees(e_head):+.1f} deg). Pushing in.')
            self.state = DockingState.FINAL_CONTACT
            self.contact_origin = self.odom
            self.contact_start = self._now()
            self._backing_off = False
            self._stop()
            return

        # Badly misaligned: turn on the spot first, there is nothing to gain
        # from driving in the wrong direction.
        if abs(e_head) > math.radians(25.0):
            self._drive(0.0, math.copysign(0.3, e_head))
            return

        # Out of room. Reverse to give the line-following law distance to work
        # with, then come in again. A real dock gets retried the same way.
        if self._backing_off:
            if e_d > 0.28:
                self._backing_off = False
                self.get_logger().info('Re-running the final approach.')
            else:
                self._drive(-self.v_final * 0.8, 0.0)
                return
        elif e_d < 0.005 and (abs(e_ct) > self.tol_lat or abs(e_head) > self.tol_angle):
            # Reached the stop distance while still off the axis. Note the
            # `and`: backing off merely because e_d is small, without checking
            # whether anything is actually wrong, turns the last centimetre of
            # a perfectly good approach into an endless shuttle.
            self._backing_off = True
            self.get_logger().warn(
                f'At the stop distance but {e_ct * 100:+.1f} cm off the axis and '
                f'{math.degrees(e_head):+.1f} deg out; reversing to line up again.')
            self._drive(-self.v_final * 0.8, 0.0)
            return

        # Line-following law. Linearised about the axis this is
        #     e_ct'   =  e_head
        #     e_head' = -(kp_head/v) e_head - (kp_lat/v) e_ct
        # so cross-track decays with length constant v / sqrt(kp_lat * v) once
        # the gains are critically damped. The default gains give roughly 0.15 m,
        # which clears a 5 cm offset over the 0.35 m run-in from the pre-dock
        # pose. Weak gains look stable but simply never converge: the robot
        # settles at a fixed offset with a compensating heading and shuttles
        # back and forth forever.
        v = clamp(0.5 * e_d, 0.025, self.v_final)
        w = clamp(self.kp_head * e_head + self.kp_lat * e_ct, -0.5, 0.5)
        # Ease off a little when steering hard, but not enough to change the
        # effective speed the gains above were tuned for.
        self._drive(v * max(0.6, 1.0 - abs(w)), w)

    def _run_final_contact(self):
        if self.odom is None or self.contact_origin is None:
            self.get_logger().warn('No odometry to dead-reckon with; ending the approach here.')
            self._stop()
            self.state = DockingState.DOCKED
            self._set_relay(True)
            return

        travelled = math.hypot(self.odom[0] - self.contact_origin[0],
                               self.odom[1] - self.contact_origin[1])
        stalled = (self.contact_start is not None
                   and (self._now() - self.contact_start) > self.contact_timeout)

        if travelled >= self.push_distance or stalled:
            self._stop()
            self.state = DockingState.DOCKED
            # Give the base back so whatever runs next can drive off the dock.
            self._set_relay(True)
            how = 'stalled against the dock' if stalled else 'clean stop'
            self.get_logger().info(f'>>> DOCKED. Final push {travelled * 100:.1f} cm ({how}). <<<')
            return
        self._drive(self.v_contact, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = DockingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
