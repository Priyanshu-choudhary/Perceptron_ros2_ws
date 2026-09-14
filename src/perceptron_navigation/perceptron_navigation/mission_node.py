#!/usr/bin/env python3
"""Patrol mission: waypoints, inspection stops, return home, dock. LEVELS 7 and 10.

    Start -> WP1 -> inspect -> WP2 -> inspect -> WP3 -> inspect -> home -> dock

Services
    /mission/start   (std_srvs/Trigger)  run the route once
    /mission/cancel  (std_srvs/Trigger)  cancel and stop
Topic
    /mission/status  (std_msgs/String)   one line of state, 2 Hz

The route lives in config/waypoints.yaml, not in this file. Check it against the
world before running: `python3 tools/checkwaypoints.py`.

Threading
---------
The mission is a long blocking sequence, so it runs on its own thread while a
MultiThreadedExecutor keeps the node's callbacks flowing in the background. That
is not decoration:

  * Timeouts are measured on the node clock, so they honour use_sim_time. The
    clock only advances if something keeps servicing /clock while the mission
    waits - which the executor thread does.
  * Running the mission inside a timer callback and calling rclpy.spin_once()
    on the same node from within it does NOT work. The node is already being
    spun, the re-entrant call processes nothing, and a loop that waits on the
    clock hangs forever. That exact bug hung this node in INSPECTING.

So: the mission thread never spins. It sleeps on the wall clock and reads state
that the executor thread has updated.

Handing over to the docking controller
--------------------------------------
Nav2 gets the robot to a staging pose about a metre in front of the dock, and
the ArUco controller takes it from there. The two must never drive at once, so
the handover disables cmd_vel_relay before calling /docking/start and re-enables
it afterwards. Without that, Nav2 keeps issuing goal-reached corrections while
the docking controller is trying to creep the last few centimetres, and the
robot oscillates in front of the dock.

Nav2 cannot do the final approach itself: its goal tolerance is 0.15 m and it
localises against a map, while docking needs about 0.02 m relative to a marker
the robot can actually see. Different sensor, different frame, different job.
"""

import math
import os
import threading
import time
from enum import Enum

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class MissionState(Enum):
    IDLE = 'IDLE'
    NAVIGATING = 'NAVIGATING'
    INSPECTING = 'INSPECTING'
    RETURNING = 'RETURNING'
    DOCKING = 'DOCKING'
    DONE = 'DONE'
    FAILED = 'FAILED'
    CANCELLED = 'CANCELLED'


def pose_stamped(navigator, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = navigator.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(float(yaw) / 2.0)
    p.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return p


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('inspect_seconds', 5.0)
        self.declare_parameter('dock_at_end', True)
        self.declare_parameter('navigation_timeout', 300.0)
        self.declare_parameter('docking_timeout', 240.0)
        self.declare_parameter('slam', True)

        self.inspect_seconds = float(self.get_parameter('inspect_seconds').value)
        self.dock_at_end = bool(self.get_parameter('dock_at_end').value)
        self.nav_timeout = float(self.get_parameter('navigation_timeout').value)
        self.dock_timeout = float(self.get_parameter('docking_timeout').value)
        self.slam = bool(self.get_parameter('slam').value)

        self.route = self._load_route(self.get_parameter('waypoints_file').value)

        self.state = MissionState.IDLE
        self.detail = ''
        self._cancel_requested = False
        self._thread = None

        cb = ReentrantCallbackGroup()
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.create_service(Trigger, '/mission/start', self._srv_start, callback_group=cb)
        self.create_service(Trigger, '/mission/cancel', self._srv_cancel, callback_group=cb)
        self.create_timer(0.5, self._publish_status, callback_group=cb)
        self.create_subscription(String, '/docking/status', self._docking_status_cb, 10,
                                 callback_group=cb)
        self._docking_state = None

        self.navigator = BasicNavigator()
        self.relay_client = self.create_client(SetBool, '/cmd_vel_relay/enable',
                                               callback_group=cb)
        self.dock_start = self.create_client(Trigger, '/docking/start', callback_group=cb)
        self.dock_cancel = self.create_client(Trigger, '/docking/cancel', callback_group=cb)
        self.detector_client = self.create_client(SetBool, '/docking/detector/enable',
                                                  callback_group=cb)

        self.get_logger().info(
            f'Mission node ready with {len(self.route["waypoints"])} waypoints. '
            'Start it with:  ros2 run perceptron_navigation mission start')
        if self.slam:
            # waypoints.yaml holds world coordinates. They only match the map
            # frame when map comes from the saved map via AMCL; under SLAM the
            # origin of map is the spawn pose, and every waypoint is offset by
            # it. WP1 ends up outside the west wall and the mission dies there.
            self.get_logger().warn(
                'Running against SLAM (slam:=true). The waypoints in '
                'waypoints.yaml are world coordinates and will be offset by the '
                'spawn pose, so this mission will very likely fail at the first '
                'waypoint. Relaunch with slam:=false to run the patrol.')

    # ------------------------------------------------------------------ setup

    def _now(self) -> float:
        """Seconds from the node clock, so timeouts honour use_sim_time.

        Wall-clock deadlines against a simulation that runs slower than real
        time silently shorten every allowance: at a real-time factor of 0.5 a
        180 s budget becomes 90 s of robot time, and journeys are cancelled
        part-way for no visible reason.
        """
        return self.get_clock().now().nanoseconds * 1e-9

    def _load_route(self, path):
        if not path:
            path = os.path.join(get_package_share_directory('perceptron_navigation'),
                                'config', 'waypoints.yaml')
        with open(path, encoding='utf-8') as handle:
            route = yaml.safe_load(handle)
        self.get_logger().info(f'Route loaded from {path}')
        return route

    def _docking_status_cb(self, msg: String):
        self._docking_state = msg.data

    def _publish_status(self):
        text = self.state.value + (f' | {self.detail}' if self.detail else '')
        self.status_pub.publish(String(data=text))

    def _set(self, state, detail=''):
        self.state, self.detail = state, detail
        self.get_logger().info(f'[{state.value}] {detail}' if detail else f'[{state.value}]')

    # --------------------------------------------------------------- services

    def _srv_start(self, request, response):
        if self._thread is not None and self._thread.is_alive():
            response.success = False
            response.message = f'Mission already running ({self.state.value}).'
            return response
        self._cancel_requested = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        response.success = True
        response.message = 'Mission started.'
        return response

    def _srv_cancel(self, request, response):
        self._cancel_requested = True
        try:
            self.navigator.cancelTask()
        except Exception:
            pass
        self._trigger(self.dock_cancel, wait=1.0)
        self._enable_relay(True)
        response.success = True
        response.message = 'Cancel requested.'
        return response

    # ------------------------------------------------------------------ utils

    def _wait_future(self, future, timeout=10.0):
        """Wait for a future from the mission thread.

        Never spin here: the executor thread owns this node, and a second
        spinner on the same node processes nothing.
        """
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _trigger(self, client, wait=5.0):
        if not client.wait_for_service(timeout_sec=wait):
            return None
        return self._wait_future(client.call_async(Trigger.Request()))

    def _enable_detector(self, enabled: bool) -> bool:
        """Run ArUco detection only when it is about to be used.

        The detector processes every camera frame through OpenCV. During a
        patrol nothing consumes the result, and on a loaded machine that work is
        enough to push Nav2's control loop past its deadline - which shows up as
        "Failed to make progress" in open space, nowhere near an obstacle.
        """
        if not self.detector_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                'ArUco detector enable service missing; it will keep running '
                'through the patrol and may starve the controller.')
            return False
        req = SetBool.Request()
        req.data = enabled
        return self._wait_future(self.detector_client.call_async(req)) is not None

    def _enable_relay(self, enabled: bool) -> bool:
        if not self.relay_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('cmd_vel_relay service missing; cannot arbitrate the base.')
            return False
        req = SetBool.Request()
        req.data = enabled
        return self._wait_future(self.relay_client.call_async(req)) is not None

    def _sleep_sim(self, seconds: float) -> bool:
        """Wait `seconds` of simulated time. False if cancelled."""
        start = self._now()
        while self._now() - start < seconds:
            if self._cancel_requested:
                return False
            time.sleep(0.05)
        return True

    # ---------------------------------------------------------------- mission

    def _run(self):
        try:
            self._execute()
        except Exception as exc:                      # noqa: BLE001
            self._set(MissionState.FAILED, f'unhandled error: {exc}')
        finally:
            self._enable_relay(True)
            self._enable_detector(True)

    def _execute(self):
        self._enable_relay(True)
        # Free the CPU that Nav2 needs; re-enabled at the docking handover.
        self._enable_detector(False)
        self._set(MissionState.NAVIGATING, 'waiting for Nav2 to become active')
        # slam_toolbox is not lifecycle managed in this stack, so there is no
        # localiser to wait on while mapping; naming bt_navigator twice makes
        # the localiser check a no-op. Waiting on amcl here would block
        # forever, because amcl only exists when slam:=false.
        localizer = 'bt_navigator' if self.slam else 'amcl'
        self.navigator.waitUntilNav2Active(localizer=localizer)

        for i, wp in enumerate(self.route['waypoints'], start=1):
            if self._cancel_requested:
                return self._set(MissionState.CANCELLED)
            name = wp.get('name', f'WP{i}')
            self._set(MissionState.NAVIGATING, f'{name} -> ({wp["x"]:.2f}, {wp["y"]:.2f})')
            if not self._goto(wp):
                return self._set(MissionState.FAILED, f'could not reach {name}')

            self._set(MissionState.INSPECTING, f'{name}, {self.inspect_seconds:.0f} s')
            if not self._inspect():
                return self._set(MissionState.CANCELLED)

        home = self.route.get('home')
        if home:
            if self._cancel_requested:
                return self._set(MissionState.CANCELLED)
            self._set(MissionState.RETURNING, f'home -> ({home["x"]:.2f}, {home["y"]:.2f})')
            if not self._goto(home):
                return self._set(MissionState.FAILED, 'could not get home')

        if self.dock_at_end and self.route.get('dock_staging'):
            if not self._dock():
                return
        self._set(MissionState.DONE, 'mission complete')

    def _goto(self, wp) -> bool:
        goal = pose_stamped(self.navigator, wp['x'], wp['y'], wp.get('yaw', 0.0))
        self.navigator.goToPose(goal)
        start = self._now()
        while not self.navigator.isTaskComplete():
            if self._cancel_requested:
                self.navigator.cancelTask()
                return False
            if self._now() - start > self.nav_timeout:
                self.navigator.cancelTask()
                self.get_logger().error(
                    f'Navigation timed out after {self.nav_timeout:.0f} s of sim time.')
                return False
        result = self.navigator.getResult()
        if result != TaskResult.SUCCEEDED:
            self.get_logger().error(f'Navigation finished with {result}')
            return False
        return True

    def _inspect(self) -> bool:
        """Placeholder for whatever the robot is actually there to do.

        Right now it holds position for inspect_seconds, which is enough for the
        ArUco detector to get clean frames or for a camera snapshot. Replace the
        body with a real task and keep the cancel check.
        """
        return self._sleep_sim(self.inspect_seconds)

    def _dock(self) -> bool:
        staging = self.route['dock_staging']
        self._set(MissionState.DOCKING, 'driving to the dock staging pose')
        if not self._goto(staging):
            self._set(MissionState.FAILED, 'could not reach the dock staging pose')
            return False

        # Hand the base over: Nav2 stops driving, the ArUco controller starts.
        self._set(MissionState.DOCKING, 'handing over to the ArUco docking controller')
        self._enable_detector(True)
        # Give the detector a moment to produce a first pose before the
        # controller's marker watchdog starts counting.
        self._sleep_sim(2.0)
        # The relay is NOT touched here any more. The docking controller now
        # hands the base to Nav2 for the approach leg and takes it back for the
        # visual final approach, so it owns the switch for the whole sequence.
        # Disabling it here would cut Nav2 off exactly when docking wants it.

        if self._trigger(self.dock_start) is None:
            self._set(MissionState.FAILED, '/docking/start did not respond')
            self._enable_relay(True)
            return False

        start = self._now()
        while self._now() - start < self.dock_timeout:
            if self._cancel_requested:
                self._trigger(self.dock_cancel, wait=1.0)
                self._enable_relay(True)
                self._set(MissionState.CANCELLED)
                return False
            if self._docking_state == 'DOCKED':
                self._set(MissionState.DONE, 'docked')
                return True
            if self._docking_state == 'FAILED':
                self._set(MissionState.FAILED, 'docking failed')
                self._enable_relay(True)
                return False
            time.sleep(0.1)

        self._trigger(self.dock_cancel, wait=1.0)
        self._enable_relay(True)
        self._set(MissionState.FAILED, 'docking timed out')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
