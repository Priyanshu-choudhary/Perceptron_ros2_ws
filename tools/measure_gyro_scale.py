#!/usr/bin/env python3
"""Measure the true gyro_scale_z by integrating /imu/data_raw over a known turn.

WHY THIS EXISTS
    gyro_params.yaml carries gyro_scale_z: 6.28 with the note "root cause in
    firmware not yet identified". An audit of the firmware finds nothing wrong:
    GYRO_SCALE_500DPS is (pi/180)/65.5, GYRO_CONFIG is 0x08 (+/-500 dps,
    65.5 LSB/dps), the offset subtraction is a bias not a gain, the EMA has
    unity DC gain, and imu.gz goes straight into the 0x56 frame.

    It matters because 6.28 scales the ERROR as well as the signal: 0.0087
    rad/s of raw noise becomes 0.055 rad/s, and a bias that wanders 3 deg/s
    between runs becomes 19 deg/s of apparent drift. That is what makes
    continuous driving unstable while short bursts stay clean.

TWO SETTINGS MUST BE CHANGED FIRST, in
src/perceptron_hardware/config/gyro_params.yaml:

    gyro_scale_z: 1.0     measure the RAW rate. Left at 6.28 you are measuring
                          the corrected stream and cannot learn anything about
                          the correction.

    track_gyro_bias: false
                          ALSO NOT OPTIONAL, and subtle. The bias tracker's
                          "am I spinning?" gate is deliberately evaluated AFTER
                          gyro_scale_z:
                              spinning = abs((gz - bias) * scale) > bias_max_rate
                          Forcing scale to 1.0 for this measurement defeats that
                          gate. A real hand spin arrives as ~0.1 rad/s raw,
                          under the 0.15 limit, so the tracker keeps running and
                          low-passes part of your genuine rotation into "bias".
                          The measured turn then comes out too small and the
                          ratio too LARGE. Measured 6.53 against a known-good
                          6.28 on the first clean run because of exactly this.

    zupt_yaw: false       THIS ONE IS NOT OPTIONAL. Zero-velocity update forces
                          the yaw rate to EXACTLY 0.0 whenever the wheels have
                          reported still for 0.3 s. Spinning the robot by hand,
                          the undriven wheels drop to zero intermittently, and
                          every one of those windows silently deletes part of
                          your rotation. The integral then comes out far too
                          small and, worse, differs run to run.

Then relaunch the bridge. No colcon build - these are symlink-installed.

PROCEDURE
    1. Mark the robot's heading on the floor.
    2. Start this script. Hold the robot still until bias calibration settles.
    3. Rotate by hand a known number of full turns, SLOWLY - aim for 10 s per
       turn. Fast spins clip at +/-500 dps and read low.
    4. Return to the mark, let it settle, Ctrl-C.

    Pass what you actually did:  --turns 1     (or 2, or 0.5)

READING THE RESULT
    The printed ratio IS the gyro_scale_z you should be using.
        ~6.28 -> the chip really does under-report; the workaround was right.
        ~1.0  -> 6.28 is spurious and yaw has been 6.28x too sensitive, which
                 would mean the STM32's fused theta in the 0x55 frame was the
                 thing that was wrong all along.
"""
import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# The MPU6050 is configured for +/-500 dps (GYRO_CONFIG = 0x08), so no honest
# reading can exceed this. Anything above it proves the stream has already been
# multiplied by gyro_scale_z, which makes it useless as a RAW measurement.
FULL_SCALE_RAD_S = math.radians(500.0)


class GyroScaleMeter(Node):
    def __init__(self, turns):
        super().__init__('measure_gyro_scale')
        self.turns = float(turns)
        self.sub = self.create_subscription(Imu, '/imu/data_raw', self._cb, 50)
        self.yaw = 0.0
        self.t_prev = None
        self.n = 0
        self.n_exact_zero = 0
        self.peak = 0.0
        self.get_logger().info('waiting for /imu/data_raw - hold the robot still')

    def _cb(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gz = float(msg.angular_velocity.z)
        if self.t_prev is not None:
            dt = t - self.t_prev
            # Same sanity window the bridge uses; skips gaps and replays.
            if 0.0 < dt < 0.5:
                self.yaw += gz * dt
                self.n += 1
                # ZUPT publishes EXACTLY 0.0, which a live gyro essentially
                # never does. Counting them detects zupt_yaw left switched on.
                if gz == 0.0:
                    self.n_exact_zero += 1
        self.t_prev = t
        self.peak = max(self.peak, abs(gz))

        if self.n and self.n % 25 == 0:
            sys.stdout.write(
                '\rintegrated %+8.3f rad (%+8.2f deg)  rate %+6.3f  '
                'peak %.3f  zeros %d/%d  n=%d'
                % (self.yaw, math.degrees(self.yaw), gz, self.peak,
                   self.n_exact_zero, self.n, self.n))
            sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--turns', type=float, default=1.0,
                    help='how many full turns you actually rotated it')
    args = ap.parse_args()

    rclpy.init()
    node = GyroScaleMeter(args.turns)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    turn = abs(node.yaw)
    expected = 2.0 * math.pi * node.turns
    zero_frac = (node.n_exact_zero / float(node.n)) if node.n else 0.0

    print('\n')
    print('samples integrated : %d' % node.n)
    print('integrated yaw     : %.4f rad (%.2f deg)'
          % (node.yaw, math.degrees(node.yaw)))
    print('peak rate seen     : %.3f rad/s (%.1f deg/s)'
          % (node.peak, math.degrees(node.peak)))
    print('exactly-zero rates : %d of %d (%.1f%%)'
          % (node.n_exact_zero, node.n, zero_frac * 100.0))
    print('')

    blocked = False

    if node.peak > FULL_SCALE_RAD_S:
        print('=' * 70)
        print('INVALID: gyro_scale_z is still applied, it is not 1.0.')
        print('  Peak was %.3f rad/s (%.0f deg/s). The MPU6050 is configured'
              % (node.peak, math.degrees(node.peak)))
        print('  for +/-500 deg/s and cannot output more than %.3f rad/s,'
              % FULL_SCALE_RAD_S)
        print('  so this stream has already been multiplied by something.')
        print('  Set gyro_scale_z: 1.0 in gyro_params.yaml and relaunch.')
        print('=' * 70)
        blocked = True

    if zero_frac > 0.05:
        print('=' * 70)
        print('INVALID: ZUPT is still on - %.1f%% of samples were EXACTLY 0.0.'
              % (zero_frac * 100.0))
        print('  A live gyro essentially never reads exactly zero. Zero-velocity')
        print('  update is forcing the rate to 0 whenever the wheels report')
        print('  still for 0.3 s, and an undriven wheel spun by hand does that')
        print('  constantly. Every such window deletes part of your rotation,')
        print('  which is why a verified 360 deg turn can integrate to a small')
        print('  fraction of itself and why two runs disagree.')
        print('  Set zupt_yaw: false in gyro_params.yaml and relaunch.')
        print('=' * 70)
        blocked = True

    if blocked:
        print('')
        print('Fix the above and re-run. Both settings live in')
        print('  src/perceptron_hardware/config/gyro_params.yaml')
        return

    if turn < 1e-6:
        print('Nothing integrated - no rotation was seen at all.')
        return

    if node.peak > 0.9 * FULL_SCALE_RAD_S:
        print('WARNING: peak reached %.0f deg/s, within 10%% of the +/-500 dps'
              % math.degrees(node.peak))
        print('         full scale. The gyro may have clipped, and a clipped')
        print('         turn always reads LOW. Redo it slower to be sure.')
        print('')

    print('you turned %.2f revolution(s) = %.4f rad' % (node.turns, expected))
    print('measured   %.4f rad' % turn)
    print('')
    print('    gyro_scale_z = %.4f / %.4f = %.3f'
          % (expected, turn, expected / turn))
    print('')
    print('Put that in gyro_params.yaml, restore zupt_yaw: true, relaunch.')


if __name__ == '__main__':
    main()
