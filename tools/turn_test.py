#!/usr/bin/env python3
"""Closed-loop turn test: rotate to 90/180/360 and check it against reality.

    python3 tools/turn_test.py

Run it with robot.launch.py up, from a second terminal. THE ROBOT SPINS IN
PLACE -- clear the space around it and keep it on the ground.

WHY CLOSED LOOP

Commanding 0.4 rad/s for 3.93 s and calling that 90 degrees assumes the robot
obeys, which it does not: it accelerates, it decelerates, the tyres scrub, and
the answer is contaminated by exactly the thing being measured. This turns
until the ESTIMATE says it has arrived, then reports what the estimate actually
reached -- overshoot included. If it stops at 91.4 degrees, it says 91.4, not
90.

WHAT IT MEASURES AGAINST

Feedback is the yaw of odom -> base_footprint, i.e. the EKF output that RViz
draws. That is deliberately the whole pipeline -- gyro, bias tracking, scale,
ZUPT, EKF -- because the question is whether what you see on screen matches
the room, not whether one sensor is internally consistent.

Rotation is accumulated UNWRAPPED, so a 360 degree target is a real full turn
and not a no-op back to the same wrapped angle.

WHAT YOU DO

After each turn it reports what it thinks it did and asks you to look at the
robot. Answer with the angle you actually measure, or just press Enter to
accept. At the end it compares the two columns and, if they disagree
consistently, tells you the scale correction that would close the gap.
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

RAD2DEG = 57.29577951308232


def yaw_of(q):
    """Yaw from a quaternion, the planar component only."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Turner(Node):
    def __init__(self, args):
        super().__init__('turn_test')
        self.args = args
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

    # ------------------------------------------------------------- plumbing

    def yaw(self):
        """Current yaw of base_footprint in odom, or None if TF is not ready."""
        try:
            t = self.buf.lookup_transform('odom', 'base_footprint',
                                          rclpy.time.Time())
        except Exception:
            return None
        return yaw_of(t.transform.rotation)

    def wait_for_tf(self, seconds=10.0):
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.yaw() is not None:
                return True
        return False

    def drive(self, wz):
        t = Twist()
        t.angular.z = float(wz)
        self.pub.publish(t)

    def stop(self, repeats=25):
        """Stop, repeatedly. One dropped zero must not leave the robot spinning."""
        for _ in range(repeats):
            self.drive(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    # ------------------------------------------------------------ the turn

    def turn(self, target_deg):
        """Rotate until the estimate reaches target_deg. Returns degrees done."""
        target = math.radians(abs(target_deg))
        sign = 1.0 if target_deg >= 0 else -1.0
        tol = math.radians(self.args.tolerance)

        last = self.yaw()
        if last is None:
            print('  no TF odom -> base_footprint')
            return None

        done = 0.0
        t0 = time.time()
        timeout = self.args.timeout

        while True:
            rclpy.spin_once(self, timeout_sec=0.02)
            y = self.yaw()
            if y is not None:
                # Unwrapped accumulation: sum the small wrapped steps rather
                # than differencing absolute angles, so passing through +-pi
                # does not register as a full turn backwards.
                done += abs(wrap_pi(y - last))
                last = y

            err = target - done
            if err <= tol:
                break
            if time.time() - t0 > timeout:
                print('  TIMEOUT after %.0f s at %.1f deg'
                      % (timeout, done * RAD2DEG))
                break

            # Proportional, with a floor. The floor exists because a big
            # geared base simply will not move below some command -- without
            # it the last few degrees are approached asymptotically and never
            # reached, and the test hangs looking like a control bug.
            wz = self.args.gain * err
            wz = max(self.args.min_rate, min(self.args.max_rate, wz))
            self.drive(sign * wz)

        self.stop()

        # Coast. The wheels keep turning briefly after the command stops, and
        # that rotation is real, so let it settle and count it.
        settle_end = time.time() + self.args.settle
        while time.time() < settle_end:
            rclpy.spin_once(self, timeout_sec=0.02)
            y = self.yaw()
            if y is not None:
                done += abs(wrap_pi(y - last))
                last = y
            self.drive(0.0)

        return done * RAD2DEG


def ask(prompt):
    try:
        return input(prompt).strip()
    except EOFError:
        return ''


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--targets', default='90,180,360',
                    help='comma-separated degrees (default 90,180,360)')
    ap.add_argument('--tolerance', type=float, default=1.5,
                    help='stop within this many degrees (default 1.5)')
    ap.add_argument('--max-rate', type=float, default=0.4,
                    help='rad/s ceiling; stay under the bridge clamp of 0.5')
    ap.add_argument('--min-rate', type=float, default=0.12,
                    help='rad/s floor so the base actually moves')
    ap.add_argument('--gain', type=float, default=1.2,
                    help='P gain, rad/s per rad of error')
    ap.add_argument('--settle', type=float, default=1.5,
                    help='seconds to keep counting coast after stopping')
    ap.add_argument('--timeout', type=float, default=90.0)
    args = ap.parse_args()

    if args.max_rate > 0.5:
        print('max-rate above the bridge clamp of 0.5 rad/s would be limited '
              'before it\nreached the motors. Use 0.4 or less.')
        return 2

    try:
        targets = [float(x) for x in args.targets.split(',') if x.strip()]
    except ValueError:
        print('--targets must be comma-separated numbers')
        return 2

    print(__doc__.split('WHY CLOSED LOOP')[0])
    print('Targets: %s degrees' % ', '.join('%g' % t for t in targets))
    print('THE ROBOT WILL SPIN IN PLACE, on the ground, with room to turn.')
    if ask('Type yes to continue: ').lower() != 'yes':
        print('aborted.')
        return 1

    rclpy.init()
    node = Turner(args)
    results = []
    try:
        if not node.wait_for_tf():
            print('\nNo odom -> base_footprint transform. Is robot.launch.py '
                  'running with ekf:=true?')
            return 1

        for target in targets:
            print('\n' + '=' * 58)
            print('TARGET: %g degrees' % target)
            print('=' * 58)
            ask('Line the robot up and note its heading. Press Enter to turn.')

            measured = node.turn(target)
            if measured is None:
                continue

            print('\n  target   : %7.1f deg' % target)
            print('  measured : %7.1f deg   (what RViz now shows)' % measured)
            print('  error    : %+7.1f deg' % (measured - target))

            print('\n  Now look at the robot.')
            reply = ask('  What angle did it ACTUALLY turn? '
                        '(number, or Enter if it matches): ')
            if reply:
                try:
                    actual = float(reply)
                except ValueError:
                    print('  not a number, recording as unconfirmed')
                    actual = None
            else:
                actual = measured
            results.append((target, measured, actual))
    except KeyboardInterrupt:
        print('\ninterrupted -- stopping the robot')
    finally:
        try:
            node.stop(40)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not results:
        return 1

    print('\n' + '=' * 58)
    print('SUMMARY')
    print('=' * 58)
    print('%-10s %-12s %-12s %-10s' % ('target', 'estimated', 'physical', 'error'))
    ratios = []
    for target, measured, actual in results:
        if actual is None:
            print('%-10.1f %-12.1f %-12s %-10s' % (target, measured, '-', '-'))
            continue
        print('%-10.1f %-12.1f %-12.1f %+-10.1f'
              % (target, measured, actual, actual - measured))
        if abs(measured) > 1.0:
            ratios.append(actual / measured)

    if ratios:
        avg = sum(ratios) / len(ratios)
        spread = max(ratios) - min(ratios)
        print('\nphysical / estimated : %.4f  (spread %.4f across %d turns)'
              % (avg, spread, len(ratios)))
        if abs(avg - 1.0) <= 0.03:
            print('-> within 3%%. The heading pipeline agrees with the room.')
        elif spread > 0.10:
            print('-> the ratio is inconsistent between turns (%.4f spread), '
                  'so this is\n   not a scale error. Suspect slip, or '
                  'imprecise physical readings.' % spread)
        else:
            print('-> consistent %.0f%% error. Multiply gyro_scale_z by %.4f:'
                  % (abs(avg - 1.0) * 100, avg))
            print('   new gyro_scale_z = current_value * %.4f' % avg)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
