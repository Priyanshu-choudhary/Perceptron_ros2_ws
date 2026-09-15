#!/usr/bin/env python3
"""
stm32_bridge_node.py
High-performance ROS 2 Hardware Bridge for Perceptron STM32 Motor Driver.
Handles:
  - Subscribes to /cmd_vel -> transmits 0xA5 0x01 TinyFrames to STM32
  - Reads 0x55 frames -> publishes /odom (twist only; pose is the EKF's job)
  - Reads 0x56 frames -> publishes /imu/data_raw (imu_link)
  - Safety watchdog: stops motors on command loss
"""

import math
import time
import struct
import serial

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


# ==============================================================================
# PROTOCOL DEFINITIONS (Matching STM32 protocol.c exactly)
# ==============================================================================
PROTO_SOF_NEW    = 0xA5
PROTO_EOF_NEW    = 0x5A

TELEMETRY_HEADER = 0x55
TELEMETRY_END    = 0x0A
TELEMETRY_SIZE   = 27
TELEMETRY_FMT    = '<IHhiiiiBB'

IMU_HEADER       = 0x56
IMU_END          = 0x0A
IMU_SIZE         = 31
IMU_FMT          = '<IffffffBB'

FRAME_CMD_VEL    = 0x01

# Raw-to-SI scaling for the 0x55 frame's power fields. These match
# Python-configurator/protocol.py exactly; if the firmware's ADC scaling
# changes, both have to change together.
#     bus_raw 24850 -> 12.42 V      cur_raw 224 -> 89.6 mA
def bus_raw_to_volts(bus_raw: int) -> float:
    return ((bus_raw >> 3) * 4) / 1000.0


def cur_raw_to_amps(cur_raw: int) -> float:
    return (cur_raw * 0.4) / 1000.0


# Odometry covariance. The ECU reports wheel velocities only, so this message
# carries no pose at all -- every pose degree of freedom is marked 1e6, which
# is how robot_localization spells "not measured". An all-zero covariance
# would instead read as infinite confidence and the EKF would believe a pose
# that is permanently pinned at the origin.
#
# Skid-steer scrubs sideways when it turns, so yaw from wheels is the weakest
# number here and is given a correspondingly loose variance. The EKF is
# configured to take only vx from this source anyway.
POSE_COV_DIAG = (1e6, 1e6, 1e6, 1e6, 1e6, 1e6)
TWIST_COV_DIAG = (0.01, 1e6, 1e6, 1e6, 1e6, 0.05)


def _diag6(diag):
    """Expand 6 diagonal terms into the row-major 36-element covariance."""
    cov = [0.0] * 36
    for i, v in enumerate(diag):
        cov[i * 6 + i] = float(v)
    return cov


def calc_xor_checksum(data: bytes) -> int:
    cs = 0
    for b in data:
        cs ^= b
    return cs


def pack_cmd_vel(linear_x: float, angular_z: float) -> bytes:
    """Builds TinyFrame 0x01 CMD_VEL."""
    payload = struct.pack('<ff', float(linear_x), float(angular_z))
    length = len(payload)
    chk = calc_xor_checksum(bytes([FRAME_CMD_VEL, length]) + payload)
    return bytes([PROTO_SOF_NEW, FRAME_CMD_VEL, length]) + payload + bytes([chk, PROTO_EOF_NEW])


class STM32BridgeNode(Node):
    def __init__(self):
        super().__init__('stm32_bridge_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 0.01)
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('imu_frame_id', 'imu_link')
        # Retained so existing launch files and hardware_params.yaml keep
        # working, but the bridge no longer has a pose to broadcast: the ECU
        # sends wheel velocities only and the EKF owns odom -> base_footprint.
        # Setting this true now only produces a warning.
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('cmd_vel_timeout', 0.3)
        # Gyro bias. Measured on this unit: gz sits at about -0.031 rad/s at
        # rest, which is -1.8 deg/s -- a full turn of heading error every three
        # and a half minutes standing still. The EKF fuses vyaw directly, so an
        # uncorrected bias walks odom -> base_footprint round in a circle and
        # SLAM tears the map apart. Averaging the first few seconds at rest and
        # subtracting it costs nothing and removes the whole effect.
        self.declare_parameter('calibrate_gyro_seconds', 5.0)  # ~500 samples at 100 Hz
        self.declare_parameter('gyro_bias_z', 0.0)  # seeds the estimate
        # The bias is not a constant. Measured on this unit across two runs it
        # moved from -0.047 to +0.006 rad/s, about 3 deg/s of swing, because
        # MEMS bias walks with temperature and time. A one-shot calibration
        # freezes whichever value happened to be there at boot, so the estimate
        # is instead refreshed continuously whenever the wheels report zero.
        self.declare_parameter('track_gyro_bias', True)
        self.declare_parameter('bias_still_seconds', 1.0)
        self.declare_parameter('bias_time_constant', 20.0)
        # "Still" cannot be decided from the wheels alone. Pick the robot up
        # and turn it by hand and the wheels report zero while the robot is
        # genuinely rotating -- the tracker would then absorb that rotation as
        # bias and the heading would sag back while you were still turning.
        # A real bias is small, so anything faster than this is real motion.
        self.declare_parameter('bias_max_rate', 0.05)   # rad/s, ~3 deg/s
        # Yaw ZUPT: while the wheels report zero, publish exactly zero yaw rate
        # instead of the gyro's noise.
        #
        # This is the encoder-gated fusion the robot actually wants. Wheel yaw
        # is worthless DURING a turn because skid-steer tyres scrub, but the
        # statement "the wheels are not turning, so the chassis is not turning"
        # is one encoders make reliably -- a stationary robot cannot slip.
        #
        # It matters here because gyro_scale_z multiplies noise as well as
        # signal: 0.0087 rad/s of raw noise becomes 0.055 rad/s (3.1 deg/s),
        # and integrating that white noise is a random walk of about 1 degree
        # per 10 s. That is the visible shake. Zeroing the rate while parked
        # removes the input to the integral entirely.
        #
        # The trade: a robot that is carried or pushed no longer updates its
        # heading, and hand-spinning it in the air shows no rotation in RViz.
        # That is the requested behaviour, but it also disables the by-hand
        # check of the gyro -- set zupt_yaw false to get that back.
        self.declare_parameter('zupt_yaw', True)
        self.declare_parameter('zupt_settle_seconds', 0.3)

        # Master IMU switch. False stops /imu/data_raw entirely -- no
        # publishing, no bias calibration, no ZUPT -- so the robot runs on
        # wheel odometry alone.
        #
        # This is not just muting a topic. ekf_real.yaml takes yaw ONLY from
        # imu0, so with the IMU silent nothing would provide rotation and the
        # robot would slide around the map without ever turning. The launch
        # therefore swaps in ekf_wheel_only.yaml, which moves vyaw onto odom0.
        # Use imu:=false on robot.launch.py rather than setting this alone.
        self.declare_parameter('use_imu', True)
        # Effective wheel separation for the wheel-derived yaw rate. Physical
        # track is 0.416 m; the effective value is larger because skid-steer
        # tyres scrub. Only matters when the EKF is fusing wheel vyaw, i.e.
        # use_imu false.
        self.declare_parameter('wheel_separation', 0.63)
        # Scale error shows up as the map turning further than the robot did.
        # Unlike the bias this really is a constant -- it is the LSB-per-dps
        # conversion in firmware, not something that drifts -- so it is
        # measured once against a physical reference angle and then left alone.
        # tools/calibrate_gyro.py does that.
        self.declare_parameter('gyro_scale_z', 1.0)

        # Hard velocity ceiling. Applied to every /cmd_vel on its way to the
        # ECU, so no teleop tool, node or joystick can exceed it -- clamping in
        # the teleop app only limits that one app.
        self.declare_parameter('max_linear_speed', 0.3)    # m/s
        self.declare_parameter('max_angular_speed', 0.5)   # rad/s

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.imu_frame = self.get_parameter('imu_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.use_imu = bool(self.get_parameter('use_imu').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)

        # Publishers & Broadcasters
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.imu_pub = (self.create_publisher(Imu, '/imu/data_raw', 20)
                        if self.use_imu else None)
        # The 0x55 frame carries bus voltage and current at 100 Hz. battery_node
        # owns the LiPo curve, the thresholds and the BatteryState message, so
        # publish the raw measurements and let it do the interpretation.
        self.volt_pub = self.create_publisher(Float32, '/battery/measured_voltage', 10)
        self.curr_pub = self.create_publisher(Float32, '/battery/measured_current', 10)
        if self.publish_tf:
            self.get_logger().warn(
                'publish_tf is set, but this bridge no longer estimates a pose -- '
                'the ECU sends wheel velocities only. odom -> base_footprint is '
                'published by the EKF; ignoring publish_tf.')

        # Subscribers & Services
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # Serial State
        self.ser = None
        self.rx_buffer = bytearray()
        self.last_cmd_time = time.time()
        self.is_stopped = True

        # Gyro bias state. Calibration only accumulates while the wheels report
        # zero velocity, so a robot that is already rolling at startup does not
        # bake its turn rate in as "zero".
        self._gyro_bias_z = float(self.get_parameter('gyro_bias_z').value)
        self._calib_seconds = float(self.get_parameter('calibrate_gyro_seconds').value)
        self._calib_done = self._gyro_bias_z != 0.0 or self._calib_seconds <= 0.0
        self._calib_sum = 0.0
        self._calib_n = 0
        self._calib_start = None
        self._moving = False
        self._still_since = None
        self._last_imu_time = None
        self._gyro_scale = float(self.get_parameter('gyro_scale_z').value)

        self.max_linear = abs(float(self.get_parameter('max_linear_speed').value))
        self.max_angular = abs(float(self.get_parameter('max_angular_speed').value))
        self._clamp_warned = 0.0
        self._last_reconnect = 0.0
        self._wheels_still_since = None
        self._zupt_active = False

        self._connect_serial()

        # Poll Timer (100 Hz for low latency UART draining)
        self.create_timer(0.01, self._poll_serial)

        # Watchdog Timer (20 Hz)
        self.create_timer(0.05, self._watchdog_check)

        self.get_logger().info(f'STM32 Bridge Node started on {self.port} @ {self.baud} baud.')
        if self.use_imu:
            self.get_logger().info(
                'IMU ON: /imu/data_raw publishing, gyro_scale_z=%.3f, '
                'zupt_yaw=%s' % (self._gyro_scale,
                                 self.get_parameter('zupt_yaw').value))
        else:
            self.get_logger().warn(
                'IMU OFF: /imu/data_raw not published. Yaw must come from the '
                'wheels -- make sure the EKF is running ekf_wheel_only.yaml, '
                'or nothing will provide rotation at all.')

    def _connect_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=self.timeout,
                write_timeout=0.1
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f'Connected to STM32 serial on {self.port}.')
        except Exception as e:
            self.get_logger().warn(f'Could not open serial port {self.port}: {e}. Will retry...')
            self.ser = None

    def _clamp(self, linear_x, angular_z):
        """Enforce the speed ceiling on the last hop before the motors.

        Deliberately here rather than in the teleop tool: this is the single
        point every command passes through, so a joystick node, a Nav2
        controller or a stray ros2 topic pub are all held to the same limit.
        """
        lx = max(-self.max_linear, min(self.max_linear, float(linear_x)))
        az = max(-self.max_angular, min(self.max_angular, float(angular_z)))

        if (lx != linear_x or az != angular_z):
            now = time.time()
            # Throttled: teleop repeats at 10 Hz and would otherwise flood.
            if now - self._clamp_warned > 2.0:
                self.get_logger().warn(
                    'cmd_vel clamped: linear %.2f->%.2f m/s, angular %.2f->%.2f rad/s'
                    % (linear_x, lx, angular_z, az))
                self._clamp_warned = now
        return lx, az

    def _cmd_vel_cb(self, msg: Twist):
        self.last_cmd_time = time.time()
        linear_x, angular_z = self._clamp(msg.linear.x, msg.angular.z)
        self.is_stopped = (abs(linear_x) < 1e-4 and abs(angular_z) < 1e-4)

        if self.ser and self.ser.is_open:
            packet = pack_cmd_vel(linear_x, angular_z)
            try:
                self.ser.write(packet)
            except Exception as e:
                self.get_logger().error(f'Serial TX Error: {e}')

    def _watchdog_check(self):
        if not self.is_stopped and (time.time() - self.last_cmd_time > self.cmd_vel_timeout):
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(pack_cmd_vel(0.0, 0.0))
                except Exception:
                    pass
            self.is_stopped = True
            self.get_logger().warn('Cmd_vel timeout! Sent zero-velocity stop to STM32.')

    def _poll_serial(self):
        if not self.ser or not self.ser.is_open:
            # Retry every 2 s. The obvious `int(time.time()) % 2 == 0` is true
            # for a whole second out of every two, and this runs at 100 Hz, so
            # it produced about a hundred reconnect attempts and a hundred
            # warning lines per second whenever the ECU was unplugged.
            nowf = time.time()
            if nowf - self._last_reconnect >= 2.0:
                self._last_reconnect = nowf
                self._connect_serial()
            return

        try:
            if self.ser.in_waiting > 0:
                chunk = self.ser.read(self.ser.in_waiting)
                if chunk:
                    self.rx_buffer.extend(chunk)

                # Process all complete packets in buffer
                while len(self.rx_buffer) > 0:
                    # 1. 0x55 Telemetry Frame
                    if self.rx_buffer[0] == TELEMETRY_HEADER:
                        if len(self.rx_buffer) < TELEMETRY_SIZE:
                            break
                        candidate = bytes(self.rx_buffer[:TELEMETRY_SIZE])
                        if candidate[-1] == TELEMETRY_END:
                            calc_chk = calc_xor_checksum(candidate[:-2])
                            if candidate[-2] == calc_chk:
                                self._parse_telemetry(candidate)
                                self.rx_buffer = self.rx_buffer[TELEMETRY_SIZE:]
                                continue
                        # Invalid frame, slide 1 byte
                        self.rx_buffer.pop(0)

                    # 2. 0x56 IMU Telemetry Frame
                    elif self.rx_buffer[0] == IMU_HEADER:
                        if len(self.rx_buffer) < IMU_SIZE:
                            break
                        candidate = bytes(self.rx_buffer[:IMU_SIZE])
                        if candidate[-1] == IMU_END:
                            calc_chk = calc_xor_checksum(candidate[:-2])
                            if candidate[-2] == calc_chk:
                                self._parse_imu(candidate)
                                self.rx_buffer = self.rx_buffer[IMU_SIZE:]
                                continue
                        self.rx_buffer.pop(0)

                    else:
                        self.rx_buffer.pop(0)

        except Exception as e:
            self.get_logger().error(f'Serial RX error: {e}')
            self.ser = None

    def _parse_telemetry(self, frame: bytes):
        _, ts, bus_raw, cur_raw, enc_l, enc_r, vel_l_mms, vel_r_mms, _, _ = struct.unpack(
            '<BIHhiiiiBB', frame
        )

        now = self.get_clock().now().to_msg()

        # Power rail. Current is negated because the ECU reports magnitude
        # while ROS wants negative-while-discharging; a positive current on a
        # driving robot tells every consumer the pack is charging.
        v = Float32()
        v.data = bus_raw_to_volts(bus_raw)
        self.volt_pub.publish(v)
        c = Float32()
        c.data = -cur_raw_to_amps(cur_raw)
        self.curr_pub.publish(c)

        # Publish Odometry. Twist only -- the pose stays at the identity with
        # a 1e6 covariance so robot_localization treats it as unmeasured.
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.orientation.w = 1.0

        # Linear velocity is average of left and right wheels
        v_linear = ((vel_l_mms + vel_r_mms) / 2000.0)
        # Angular velocity from the wheel speed difference. With use_imu false
        # this is the ONLY yaw source the EKF has, which makes wheel_separation
        # the calibration constant for the whole heading estimate -- the
        # counterpart of gyro_scale_z on the IMU path.
        #
        # The right value is not the physical track width. A skid-steer scrubs
        # its tyres sideways to turn, so it behaves as though its wheels were
        # further apart than they are; if turns come out consistently short in
        # wheel-only mode, raise this.
        v_angular = ((vel_r_mms - vel_l_mms) / 1000.0) / self.wheel_separation

        odom.twist.twist.linear.x = float(v_linear)
        odom.twist.twist.angular.z = float(v_angular)

        odom.pose.covariance = _diag6(POSE_COV_DIAG)
        odom.twist.covariance = _diag6(TWIST_COV_DIAG)

        # Gate gyro calibration on the wheels actually being still.
        self._moving = (vel_l_mms != 0 or vel_r_mms != 0)

        # ZUPT engages only after the wheels have been still for a settling
        # period, but disengages on the very first sign of motion. Asymmetric
        # on purpose: a late engage costs nothing, whereas a late disengage
        # would swallow the start of a real turn.
        nowf = time.time()
        if self._moving:
            self._wheels_still_since = None
            self._zupt_active = False
        else:
            if self._wheels_still_since is None:
                self._wheels_still_since = nowf
            settle = float(self.get_parameter('zupt_settle_seconds').value)
            self._zupt_active = (
                bool(self.get_parameter('zupt_yaw').value)
                and (nowf - self._wheels_still_since) >= settle)

        self.odom_pub.publish(odom)

    def _calibrate_gyro(self, gz):
        """Initial bias estimate. Returns True while still calibrating."""
        if self._calib_done:
            return False

        nowf = time.time()
        if self._calib_start is None:
            self._calib_start = nowf
            self.get_logger().info(
                'calibrating gyro bias, hold the robot still for %.1f s'
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
                self._gyro_bias_z = self._calib_sum / self._calib_n
            self._calib_done = True
            self.get_logger().info(
                'gyro bias_z = %+.5f rad/s (%+.3f deg/s) from %d samples'
                % (self._gyro_bias_z, self._gyro_bias_z * 180.0 / math.pi,
                   self._calib_n))
        return not self._calib_done

    def _track_bias(self, gz, dt):
        """Keep following the bias whenever the robot is standing still.

        A single boot-time number goes stale: this gyro's bias moved about
        3 deg/s between two runs, and an error that size walks the heading a
        full turn every two minutes. While the wheels report zero the true
        yaw rate is zero by definition, so whatever the gyro reads is bias and
        can be low-passed into the estimate.

        The time constant is deliberately long. Following too eagerly would
        absorb genuine slow rotation -- being pushed, or a slipping track -- as
        if it were bias, and then subtract real motion.
        """
        if not bool(self.get_parameter('track_gyro_bias').value):
            return

        nowf = time.time()

        # Two independent conditions, because either alone is fooled: the
        # wheels miss a robot being carried, and the gyro alone would treat a
        # genuinely huge bias as motion and never converge.
        # Compare in REAL rad/s, i.e. after the scale factor, not in the raw
        # units the sensor happens to report. With a large gyro_scale_z the
        # raw reading for a genuine turn is small -- at scale 6.28 a real
        # 0.4 rad/s turn arrives as 0.064 raw -- and a gate applied to the raw
        # value would treat most real rotation as bias and absorb it.
        rate_limit = abs(float(self.get_parameter('bias_max_rate').value))
        spinning = abs((gz - self._gyro_bias_z) * self._gyro_scale) > rate_limit

        if self._moving or spinning:
            self._still_since = None
            return

        if self._still_since is None:
            self._still_since = nowf
            return

        # Let the chassis settle before believing "still": a robot that just
        # braked is still rocking on its suspension.
        if nowf - self._still_since < float(self.get_parameter('bias_still_seconds').value):
            return

        tau = max(1e-3, float(self.get_parameter('bias_time_constant').value))
        alpha = min(1.0, dt / tau)
        self._gyro_bias_z += alpha * (gz - self._gyro_bias_z)

    def _parse_imu(self, frame: bytes):
        _, ts, ax, ay, az, gx, gy, gz, _, _ = struct.unpack('<BIffffffBB', frame)

        # IMU disabled. The frame is still decoded rather than skipped, because
        # the byte stream is shared with the 0x55 telemetry and dropping 31
        # bytes without parsing them would desynchronise the reader.
        if not self.use_imu:
            return

        # Publish nothing at all while calibrating. Emitting a biased vyaw for
        # the first few seconds would let the EKF integrate exactly the error
        # this is meant to remove.
        if self._calibrate_gyro(gz):
            return

        nowf = time.time()
        dt = 0.01 if self._last_imu_time is None else (nowf - self._last_imu_time)
        self._last_imu_time = nowf
        self._track_bias(gz, dt)

        # Bias first, then scale: the bias is an additive offset on the raw
        # reading, so scaling before removing it would scale the offset too.
        gz = (gz - self._gyro_bias_z) * self._gyro_scale

        # ZUPT. Report the zero the encoders are asserting, not the gyro's
        # noise around it, and tell the EKF the zero is trustworthy so it
        # pins the heading rather than averaging the two.
        if self._zupt_active:
            gz = 0.0
            yaw_var = 1e-6
        else:
            # Measured: 0.0087 rad/s raw std, x6.28 scale -> 0.055 rad/s, so
            # the true variance is about 0.003. Declaring 0.01 overstates the
            # noise roughly threefold, which makes the EKF smooth harder --
            # the right direction when the sensor is this noisy.
            yaw_var = 0.01

        now = self.get_clock().now().to_msg()
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.imu_frame

        # Linear acceleration (m/s^2)
        imu_msg.linear_acceleration.x = float(ax)
        imu_msg.linear_acceleration.y = float(ay)
        imu_msg.linear_acceleration.z = float(az)

        # Angular velocity (rad/s)
        imu_msg.angular_velocity.x = float(gx)
        imu_msg.angular_velocity.y = float(gy)
        imu_msg.angular_velocity.z = float(gz)

        # Same reasoning as the odometry covariance: all-zero means "certain",
        # not "unknown". The EKF fuses only vyaw from this message. A leading
        # -1 in orientation_covariance is the REP-145 way to say this message
        # carries no absolute orientation at all, which is true -- there is no
        # magnetometer, so heading here is an unbounded gyro integral.
        imu_msg.orientation_covariance = [-1.0] + [0.0] * 8
        imu_msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, yaw_var]
        imu_msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04]

        self.imu_pub.publish(imu_msg)


def main(args=None):
    rclpy.init(args=args)
    node = STM32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # SIGTERM. This is how ros2 launch stops its nodes, so without this
        # every Ctrl-C on the launch printed a traceback and made a normal
        # shutdown look like a crash.
        pass
    finally:
        if node.ser and node.ser.is_open:
            try:
                node.ser.write(pack_cmd_vel(0.0, 0.0))
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        # A SIGTERM during spin can already have torn the context down, and
        # calling shutdown twice raises RCLError -- which then buries the real
        # reason the node stopped under an unrelated traceback.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
