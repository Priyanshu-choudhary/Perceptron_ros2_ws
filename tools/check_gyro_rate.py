#!/usr/bin/env python3
"""Decide the gyro scale question without measuring any angle.

Turning through a marked angle needs you to be sure how far the robot actually
went, and a mis-executed turn produces a confident wrong scale factor. This
test avoids the problem entirely: command a known yaw RATE and read back what
the gyro reports while it holds it.

    scale = commanded_rate / measured_rate

The STM32 closes its yaw loop on the wheel encoders, so the commanded rate is
achieved by the motors and is independent of the gyro. That makes the command
an honest reference for the gyro's units. It does depend on the drivetrain
actually reaching the setpoint, so the robot must be ON THE GROUND with grip --
not up on blocks, where the wheels spin free and the chassis never turns.

Run it with the full stack up (robot.launch.py), from another terminal:

    python3 tools/check_gyro_rate.py

THE ROBOT WILL SPIN IN PLACE. Clear space around it first.
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu

RAD2DEG = 57.29577951308232


class RateCheck(Node):
    def __init__(self, rate, seconds, settle):
        super().__init__('gyro_rate_check')
        self.rate = rate
        self.seconds = seconds
        self.settle = settle
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Imu, '/imu/data_raw', self._imu_cb, 50)
        self.samples = []
        self.collecting = False
        self.saw_imu = False

    def _imu_cb(self, msg):
        self.saw_imu = True
        if self.collecting:
            self.samples.append(msg.angular_velocity.z)

    def _drive(self, wz):
        t = Twist()
        t.angular.z = float(wz)
        self.pub.publish(t)

    def run(self):
        # The bridge watchdog zeroes the motors after 0.3 s without a command,
        # so the command has to be repeated for the whole test, not sent once.
        deadline = time.time() + 2.0
        while time.time() < deadline and not self.saw_imu:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not self.saw_imu:
            self.get_logger().error(
                'no /imu/data_raw. Is robot.launch.py running?')
            return None

        print('spinning up to %.2f rad/s (%.1f deg/s), settling %.1f s...'
              % (self.rate, self.rate * RAD2DEG, self.settle))
        t_end = time.time() + self.settle
        while time.time() < t_end:
            self._drive(self.rate)
            rclpy.spin_once(self, timeout_sec=0.02)

        print('measuring for %.1f s...' % self.seconds)
        self.collecting = True
        t_end = time.time() + self.seconds
        while time.time() < t_end:
            self._drive(self.rate)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.collecting = False

        # Stop, and keep saying so: one zero can be missed, and a robot that
        # keeps spinning because a single message dropped is not acceptable.
        for _ in range(25):
            self._drive(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
        print('stopped.')
        return self.samples


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rate', type=float, default=0.4,
                    help='commanded yaw rate rad/s (default 0.4, under the '
                         '0.5 ceiling so the clamp does not distort it)')
    ap.add_argument('--seconds', type=float, default=6.0)
    ap.add_argument('--settle', type=float, default=2.0)
    args = ap.parse_args()

    if abs(args.rate) > 0.5:
        print('rate above the 0.5 rad/s clamp would be limited before it '
              'reached the motors,\nwhich invalidates the comparison. Use '
              '0.4 or less.')
        return 2

    print(__doc__.split('Run it with')[0])
    print('THE ROBOT WILL SPIN IN PLACE. It must be on the ground, with room '
          'to turn.')
    try:
        if input('Type yes to continue: ').strip().lower() != 'yes':
            print('aborted.')
            return 1
    except EOFError:
        print('needs an interactive terminal.')
        return 1

    rclpy.init()
    node = RateCheck(args.rate, args.seconds, args.settle)
    try:
        samples = node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not samples:
        return 1

    n = len(samples)
    mean = sum(samples) / n
    std = (sum((s - mean) ** 2 for s in samples) / n) ** 0.5
    print('\n  commanded : %+.4f rad/s  (%+.2f deg/s)'
          % (args.rate, args.rate * RAD2DEG))
    print('  measured  : %+.4f rad/s  (%+.2f deg/s)  std %.4f, n=%d'
          % (mean, mean * RAD2DEG, std, n))

    if abs(mean) < 1e-3:
        print('\n  the gyro read essentially zero. Either the robot did not '
              'turn (wheels\n  off the ground? no traction?) or the IMU is '
              'not reporting.')
        return 1

    scale = args.rate / abs(mean)
    print('  implied scale : %.3f' % scale)
    print('')
    if abs(scale - 1.0) <= 0.15:
        print('  VERDICT: scale is ~1.0. Leave gyro_scale_z at 1.0.')
        print('  Any remaining heading error is bias, not scale.')
    else:
        print('  VERDICT: the gyro disagrees with the commanded rate by '
              '%.0f%%.' % (abs(scale - 1.0) * 100))
        print('  Repeat at a different rate (--rate 0.2 and 0.3). A real '
              'scale error gives\n  the SAME ratio at every rate; a '
              'drivetrain that misses its setpoint does not.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
