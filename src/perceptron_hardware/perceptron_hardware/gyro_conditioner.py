"""Yaw-gyro conditioning: bias calibration, bias tracking, scale, and ZUPT.

WHY THIS IS A SHARED MODULE
---------------------------
This logic used to live inside stm32_bridge_node only. jetson_bridge_node
publishes the same /imu/data_raw from the same MPU-6050 over the ZeroMQ LAN
bridge, but had none of it: no bias, no scale, no ZUPT, and an all-zero
angular_velocity_covariance. robot_localization reads an all-zero covariance as
"certain", substitutes ~1e-9, and pins the yaw Kalman gain to 1.0 - so on that
path the EKF replayed a raw, biased, unscaled gyro and the wheels never got a
vote.

robot.launch.py defaults to use_jetson:=true, so that was the DEFAULT path.
Every gyro fix in the codebase was being bypassed by the node actually running.

Two copies of this logic is what created that gap, so there is now one copy and
both bridges call it. If you add a correction here, both paths get it.

ORDER OF OPERATIONS (do not rearrange)
--------------------------------------
    raw gz -> [calibrate] -> subtract bias -> multiply scale -> [ZUPT] -> publish

Bias before scale: the bias is an additive offset on the RAW reading, so scaling
first would scale the offset too and leave a residual behind.
"""

import math
import time


class GyroConditioner:
    """Turns a raw yaw-rate reading into one the EKF can trust.

    The caller owns the ROS plumbing; this owns the signal. Feed it wheel motion
    from the odometry path and raw gz from the IMU path.
    """

    #: (name, default) for every parameter this needs. Both bridges declare the
    #: same set from here, so the two nodes cannot drift apart again.
    PARAMS = (
        ('calibrate_gyro_seconds', 5.0),   # ~500 samples at 100 Hz
        ('gyro_bias_z', 0.0),              # non-zero seeds and skips calibration
        ('track_gyro_bias', True),
        ('bias_still_seconds', 1.0),
        ('bias_time_constant', 20.0),
        ('bias_max_rate', 0.05),           # rad/s, ~3 deg/s
        ('zupt_yaw', True),
        ('zupt_settle_seconds', 0.3),
        ('gyro_scale_z', 1.0),
    )

    #: Variance published on vyaw while ZUPT holds the rate at exactly zero.
    #: Tiny on purpose: the encoders are asserting this zero and the EKF should
    #: pin the heading to it rather than average it against gyro noise.
    ZUPT_YAW_VARIANCE = 1e-6

    #: Variance published on vyaw normally. Measured 0.0087 rad/s of raw noise;
    #: at scale 6.28 that is 0.055 rad/s, so true variance is about 0.003.
    #: Declaring 0.01 overstates it roughly threefold, which makes the EKF smooth
    #: harder - the right direction for a sensor this noisy.
    YAW_VARIANCE = 0.01

    @classmethod
    def declare_parameters(cls, node):
        """Declare every parameter this needs on `node`. Safe to call once."""
        for name, default in cls.PARAMS:
            if not node.has_parameter(name):
                node.declare_parameter(name, default)

    def __init__(self, node):
        self._node = node
        self._log = node.get_logger()

        self._scale = float(self._p('gyro_scale_z'))
        self._bias = float(self._p('gyro_bias_z'))

        self._calib_seconds = float(self._p('calibrate_gyro_seconds'))
        # A pre-seeded bias means someone measured it already; do not re-average.
        self._calib_done = self._bias != 0.0 or self._calib_seconds <= 0.0
        self._calib_start = None
        self._calib_sum = 0.0
        self._calib_n = 0

        self._moving = True          # assume moving until the wheels say otherwise
        self._wheels_still_since = None
        self._still_since = None
        self._zupt_active = False
        self._last_imu_time = None

    def _p(self, name):
        return self._node.get_parameter(name).value

    # ------------------------------------------------------------------ wheels

    def set_wheel_motion(self, moving):
        """Call from the ODOMETRY path with whether the wheels are turning.

        The wheels, not the gyro, decide what "still" means. Asking the gyro
        whether the gyro is stationary is circular and would happily lock in
        whatever the robot was doing at the time.

        ZUPT engages only after the wheels have been still for a settling
        period, but disengages on the very first sign of motion. Asymmetric on
        purpose: a late engage costs nothing, whereas a late disengage would
        swallow the start of a real turn.
        """
        self._moving = bool(moving)
        nowf = time.time()

        if self._moving:
            self._wheels_still_since = None
            self._zupt_active = False
            return

        if self._wheels_still_since is None:
            self._wheels_still_since = nowf

        self._zupt_active = (
            bool(self._p('zupt_yaw'))
            and (nowf - self._wheels_still_since) >= float(self._p('zupt_settle_seconds')))

    # --------------------------------------------------------------------- imu

    def condition(self, gz):
        """Call from the IMU path with the raw yaw rate.

        Returns (yaw_rate, yaw_variance), or None while still calibrating - and
        when it returns None the caller must publish NOTHING. Emitting a biased
        vyaw for the first few seconds would let the EKF integrate exactly the
        error this is here to remove.
        """
        if self._calibrate(gz):
            return None

        nowf = time.time()
        dt = 0.01 if self._last_imu_time is None else (nowf - self._last_imu_time)
        self._last_imu_time = nowf
        self._track_bias(gz, dt)

        gz = (gz - self._bias) * self._scale

        if self._zupt_active:
            return 0.0, self.ZUPT_YAW_VARIANCE
        return gz, self.YAW_VARIANCE

    def angular_velocity_covariance(self, yaw_var):
        """Row-major 3x3 for sensor_msgs/Imu.angular_velocity_covariance.

        Never leave this all-zero. All-zero means "certain", not "unknown".
        """
        return [0.02, 0.0, 0.0,
                0.0, 0.02, 0.0,
                0.0, 0.0, float(yaw_var)]

    # ------------------------------------------------------------- calibration

    def _calibrate(self, gz):
        """Initial bias estimate. Returns True while still calibrating."""
        if self._calib_done:
            return False

        nowf = time.time()
        if self._calib_start is None:
            self._calib_start = nowf
            self._log.info('calibrating gyro bias, hold the robot still for %.1f s'
                           % self._calib_seconds)

        if self._moving:
            # Movement during calibration invalidates the samples so far.
            self._calib_sum = 0.0
            self._calib_n = 0
            self._calib_start = nowf
            return True

        self._calib_sum += gz
        self._calib_n += 1

        if nowf - self._calib_start >= self._calib_seconds:
            if self._calib_n:
                self._bias = self._calib_sum / self._calib_n
            self._calib_done = True
            self._log.info('gyro bias_z = %+.5f rad/s (%+.3f deg/s) from %d samples'
                           % (self._bias, math.degrees(self._bias), self._calib_n))
        return not self._calib_done

    def _track_bias(self, gz, dt):
        """Keep following the bias whenever the robot is standing still.

        A single boot-time number goes stale: this gyro's bias moved about
        3 deg/s between two runs, and an error that size walks the heading a
        full turn every two minutes. While the wheels report zero the true yaw
        rate is zero by definition, so whatever the gyro reads is bias and can
        be low-passed into the estimate.

        The time constant is deliberately long. Following too eagerly would
        absorb genuine slow rotation - being pushed, or a slipping track - as if
        it were bias, and then subtract real motion.
        """
        if not bool(self._p('track_gyro_bias')):
            return

        nowf = time.time()

        # Two independent conditions, because either alone is fooled: the wheels
        # miss a robot being carried, and the gyro alone would treat a genuinely
        # huge bias as motion and never converge.
        # Compare in REAL rad/s, i.e. AFTER the scale factor, not in the raw
        # units the sensor happens to report. With a large gyro_scale_z the raw
        # reading for a genuine turn is small - at scale 6.28 a real 0.4 rad/s
        # turn arrives as 0.064 raw - and a gate applied to the raw value would
        # treat most real rotation as bias and absorb it.
        rate_limit = abs(float(self._p('bias_max_rate')))
        spinning = abs((gz - self._bias) * self._scale) > rate_limit

        if self._moving or spinning:
            self._still_since = None
            return

        if self._still_since is None:
            self._still_since = nowf
            return

        # Let the chassis settle before believing "still": a robot that just
        # braked is still rocking on its suspension.
        if nowf - self._still_since < float(self._p('bias_still_seconds')):
            return

        tau = max(1e-3, float(self._p('bias_time_constant')))
        alpha = min(1.0, dt / tau)
        self._bias += alpha * (gz - self._bias)

    # ---------------------------------------------------------------- introspection

    @property
    def bias(self):
        return self._bias

    @property
    def scale(self):
        return self._scale

    @property
    def calibrated(self):
        return self._calib_done

    @property
    def zupt_active(self):
        return self._zupt_active
