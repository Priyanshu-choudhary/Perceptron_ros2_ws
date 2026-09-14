"""Reject corrupt sensor samples before they reach the EKF.

WHY THIS EXISTS
---------------
robot_localization cannot recover from a NaN. Once a NaN enters the state or the
covariance, every later prediction is NaN, the filter stops publishing a usable
odom -> base_footprint, the robot disappears from RViz, and the ONLY fix is
restarting the node. One bad float costs you the whole run.

Both bridges could produce one:

  * stm32_bridge_node unpacks six floats straight out of a UART frame. The XOR
    checksum is 8 bits, so roughly 1 in 256 corrupt frames passes it.
  * jetson_bridge_node unpacks msgpack off a ZeroMQ socket over Wi-Fi, and
    trusts whatever floats come back with no bounds at all.

The EKF fuses exactly two numbers from these nodes - twist.linear.x from the
odometry and angular_velocity.z from the IMU - so those two are the poisoning
route and are checked hardest. The rest is hygiene.

Limits are physical, not statistical: an MPU-6050 at +/-500 dps CANNOT report
more than 8.73 rad/s, so anything beyond that is corruption by definition and
throwing it away costs no real data. Note these bound the RAW reading, before
gyro_scale_z is applied.
"""

import math


#: Physical ceilings for a raw MPU-6050 sample at the ranges the firmware sets
#: (+/-500 dps gyro, +/-4 g accel), with headroom for the bias offset.
GYRO_LIMIT = 12.0        # rad/s  (range is 8.73)
ACCEL_LIMIT = 50.0       # m/s^2  (range is 39.2)

#: Ceilings for the drivetrain. max_linear_speed is 0.3 m/s and max_angular
#: 0.5 rad/s, so these are an order of magnitude of slack - they catch
#: corruption, not aggressive driving.
LINEAR_LIMIT = 5.0       # m/s
ANGULAR_LIMIT = 10.0     # rad/s

#: A plausible bound on how far the robot can be from where odometry started.
POSITION_LIMIT = 1.0e4   # m


def finite(*values):
    """True when every value is a real number - no NaN, no inf."""
    for v in values:
        if v is None:
            return False
        try:
            f = float(v)
        except (TypeError, ValueError):
            return False
        if math.isnan(f) or math.isinf(f):
            return False
    return True


def within(limit, *values):
    """True when every value is finite AND within +/-limit."""
    if not finite(*values):
        return False
    return all(abs(float(v)) <= limit for v in values)


class RejectCounter:
    """Counts rejected samples and logs at most once a second.

    Silent dropping hides a degrading radio link or a failing I2C bus; logging
    every bad frame at 100 Hz floods the console and is its own outage. So:
    drop silently, but report a running total once a second.
    """

    def __init__(self, node, label):
        self._log = node.get_logger()
        self._clock = node.get_clock()
        self._label = label
        self.total = 0
        self._since_report = 0
        self._last_report = None

    def reject(self, detail=''):
        """Record one rejected sample. Always returns False, so callers can
        write `return counter.reject('gyro out of range')`."""
        self.total += 1
        self._since_report += 1

        now = self._clock.now().nanoseconds * 1e-9
        if self._last_report is None:
            self._last_report = now
        elif now - self._last_report >= 1.0:
            self._log.warn(
                '%s: dropped %d corrupt sample(s) in the last %.1f s (%d total)%s'
                % (self._label, self._since_report, now - self._last_report,
                   self.total, (' - ' + detail) if detail else ''))
            self._last_report = now
            self._since_report = 0
        return False


def imu_sample_ok(ax, ay, az, gx, gy, gz):
    """True when a raw IMU sample is physically possible."""
    return (within(ACCEL_LIMIT, ax, ay, az)
            and within(GYRO_LIMIT, gx, gy, gz))


def odom_sample_ok(v_linear, v_angular, x=0.0, y=0.0, yaw=0.0):
    """True when a raw odometry sample is physically possible."""
    return (within(LINEAR_LIMIT, v_linear)
            and within(ANGULAR_LIMIT, v_angular)
            and within(POSITION_LIMIT, x, y)
            and finite(yaw))
