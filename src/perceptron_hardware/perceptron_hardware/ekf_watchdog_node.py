#!/usr/bin/env python3
"""Detect a poisoned or diverged EKF and reset it in place.

WHY THIS EXISTS
---------------
robot_localization has no NaN recovery. Once a NaN reaches the state vector or
the covariance, every subsequent predict/correct step produces NaN, the filter
stops publishing a usable odom -> base_footprint, the robot vanishes from RViz,
and the run is over until someone restarts the node by hand.

sensor_guard.py stops the known route in (corrupt UART frames, corrupt msgpack
off the Wi-Fi link). This is the backstop for everything else: an unmodelled
input, a numerical blow-up during a long run, or a source added later that
forgets to validate. It turns "the robot disappeared, restart everything" into a
one-line warning and a half-second gap.

HOW IT RECOVERS
---------------
robot_localization subscribes to /set_pose (geometry_msgs/PoseWithCovarianceStamped)
and reinitialises its state and covariance from whatever arrives. The watchdog
replays the last pose it saw that was finite, so the robot resumes roughly where
it was rather than teleporting to the origin mid-run.

WHAT IT DELIBERATELY DOES NOT DO
--------------------------------
It does not reset on silence. No messages can mean the bridge died, the robot is
disarmed, or the serial cable fell out - resetting the filter fixes none of those
and would hide them. Silence gets a warning; only a genuinely bad NUMBER gets a
reset.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def _all_finite(values):
    for v in values:
        if math.isnan(v) or math.isinf(v):
            return False
    return True


class EkfWatchdog(Node):

    def __init__(self):
        super().__init__('ekf_watchdog')

        self.declare_parameter('filtered_topic', '/odometry/filtered')
        self.declare_parameter('set_pose_topic', '/set_pose')
        # Variance at which to WARN - never to reset. This filter fuses vx and
        # vyaw and nothing absolute, so its position variance grows without
        # bound by design: that is what dead reckoning is, not divergence.
        # Resetting on it would fire on every healthy long run, which is
        # exactly the bug this parameter used to cause. NaN is the only
        # condition that actually warrants a reset.
        self.declare_parameter('warn_position_variance', 1.0e6)
        # Never reset faster than this. A reset that immediately re-poisons is a
        # loop that floods the log and hides the real fault.
        self.declare_parameter('min_reset_interval', 2.0)
        # Warn (do not reset) if the filter goes quiet this long.
        self.declare_parameter('silence_warn_seconds', 2.0)

        self._set_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter('set_pose_topic').value, 1)

        self.create_subscription(
            Odometry, self.get_parameter('filtered_topic').value, self._on_odom, 10)

        self._last_good = None          # (x, y, qz, qw)
        self._last_msg_time = None
        self._last_reset = None
        self._resets = 0
        self._warned_silent = False
        self._warned_variance = False

        self.create_timer(0.5, self._check_silence)
        self.get_logger().info(
            'ekf_watchdog up: watching %s, will reset via %s'
            % (self.get_parameter('filtered_topic').value,
               self.get_parameter('set_pose_topic').value))

    # ------------------------------------------------------------------ checks

    def _on_odom(self, msg):
        self._last_msg_time = self._now()
        self._warned_silent = False

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist

        fields = [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                  t.linear.x, t.linear.y, t.angular.z]

        if not _all_finite(fields) or not _all_finite(list(msg.pose.covariance)):
            self._reset('filter output contains NaN or inf')
            return

        var_x = msg.pose.covariance[0]
        var_y = msg.pose.covariance[7]
        limit = float(self.get_parameter('warn_position_variance').value)
        if (var_x > limit or var_y > limit) and not self._warned_variance:
            self._warned_variance = True
            self.get_logger().warn(
                'odom-frame position variance is %.3g / %.3g. Growth is expected '
                '(nothing absolute is fused here) - this is only worth a look if '
                'it climbed suddenly rather than gradually.' % (var_x, var_y))

        # Healthy sample: remember it as the place to resume from.
        self._last_good = (p.x, p.y, q.z, q.w)

    def _check_silence(self):
        if self._last_msg_time is None:
            return
        quiet = self._now() - self._last_msg_time
        limit = float(self.get_parameter('silence_warn_seconds').value)
        if quiet > limit and not self._warned_silent:
            self._warned_silent = True
            # Deliberately a warning, not a reset - see the module docstring.
            self.get_logger().warn(
                'no filtered odometry for %.1f s. The EKF is not producing output: '
                'check that the bridge is alive and publishing /odom and '
                '/imu/data_raw, and that the robot is armed.' % quiet)

    # ----------------------------------------------------------------- recovery

    def _reset(self, why):
        now = self._now()
        interval = float(self.get_parameter('min_reset_interval').value)
        if self._last_reset is not None and (now - self._last_reset) < interval:
            return
        self._last_reset = now
        self._resets += 1

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        if self._last_good is not None:
            x, y, qz, qw = self._last_good
            where = 'last good pose (%.2f, %.2f)' % (x, y)
        else:
            x, y, qz, qw = 0.0, 0.0, 0.0, 1.0
            where = 'the origin (no good pose seen yet)'

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # Modest, honest uncertainty: the robot is near where it was, but the
        # filter just failed, so do not claim precision it has not got.
        for i, v in ((0, 0.25), (7, 0.25), (14, 0.25),
                     (21, 0.25), (28, 0.25), (35, 0.5)):
            msg.pose.covariance[i] = v

        self._set_pose_pub.publish(msg)
        self.get_logger().error(
            'EKF RESET #%d: %s. Reinitialised at %s. If this repeats, a sensor '
            'is feeding bad numbers - check the bridge log for dropped samples.'
            % (self._resets, why, where))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = EkfWatchdog()
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
