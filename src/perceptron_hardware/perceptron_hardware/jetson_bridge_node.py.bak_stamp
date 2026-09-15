#!/usr/bin/env python3
"""
jetson_bridge_node.py
ROS 2 Humble Client Node communicating with Jetson Nano over local Wi-Fi via ZeroMQ.
Translates binary ZMQ streams into native ROS 2 topics:
  - Publishes /scan (LaserScan) from LD19
  - Publishes /odom (Odometry) from STM32
  - Publishes /imu/data_raw (Imu) from STM32
  - Publishes /battery/measured_voltage and /battery/measured_current
  - Subscribes to /cmd_vel (Twist) and sends commands to Jetson
"""

import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool
import tf2_ros
import zmq
import msgpack

from perceptron_hardware.gyro_conditioner import GyroConditioner
from perceptron_hardware import sensor_guard

POSE_COV_DIAG = (0.05, 0.05, 1e6, 1e6, 1e6, 0.10)
TWIST_COV_DIAG = (0.01, 1e6, 1e6, 1e6, 1e6, 0.05)


def _diag6(diag):
    cov = [0.0] * 36
    for i, v in enumerate(diag):
        cov[i * 6 + i] = float(v)
    return cov


def yaw_to_quaternion(yaw: float) -> tuple:
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class JetsonBridgeNode(Node):
    def __init__(self):
        super().__init__('jetson_bridge_node')

        # Declare parameters
        self.declare_parameter('jetson_ip', '192.168.1.11')
        self.declare_parameter('telemetry_port', 5555)
        self.declare_parameter('cmd_port', 5556)
        self.declare_parameter('laser_frame_id', 'laser_frame')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('use_imu', True)
        # The same gyro conditioning stm32_bridge_node applies. Without this the
        # LAN-bridge path published a raw, biased, unscaled gyro with an all-zero
        # covariance - and since robot.launch.py defaults to use_jetson:=true,
        # that was the path the robot actually ran.
        GyroConditioner.declare_parameters(self)

        self.jetson_ip = self.get_parameter('jetson_ip').value
        self.telemetry_port = self.get_parameter('telemetry_port').value
        self.cmd_port = self.get_parameter('cmd_port').value
        self.laser_frame_id = self.get_parameter('laser_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.auto_arm = self.get_parameter('auto_arm').value
        self.use_imu = bool(self.get_parameter('use_imu').value)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_odom_time = None
        self.gyro = GyroConditioner(self)
        # msgpack over ZeroMQ over Wi-Fi carries no application checksum, so
        # every float here is untrusted. A single NaN reaching the EKF poisons
        # it for the rest of the run - there is no recovery, only a restart.
        self._imu_rejects = sensor_guard.RejectCounter(self, 'imu')
        self._odom_rejects = sensor_guard.RejectCounter(self, 'odom')
        self.is_armed = False

        # ROS 2 Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 20)
        self.volt_pub = self.create_publisher(Float32, '/battery/measured_voltage', 10)
        self.curr_pub = self.create_publisher(Float32, '/battery/measured_current', 10)
        self.status_pub = self.create_publisher(String, '/jetson/status', 10)

        # TF Broadcaster
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Services for Arm/Disarm
        self.create_service(SetBool, '/arm_motors', self._arm_motors_cb)
        self.create_service(SetBool, '/robot/arm', self._arm_motors_cb)

        # ROS 2 Subscriber for /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        # ZeroMQ Setup
        self.zmq_context = zmq.Context()

        # Telemetry SUB socket
        self.sub_socket = self.zmq_context.socket(zmq.SUB)
        self.sub_socket.setsockopt(zmq.SUBSCRIBE, b"")  # Subscribe to all topics
        self.sub_socket.setsockopt(zmq.RCVTIMEO, 1000)   # 1s timeout
        telemetry_url = f"tcp://{self.jetson_ip}:{self.telemetry_port}"
        self.sub_socket.connect(telemetry_url)
        self.get_logger().info(f"Connected to Jetson telemetry at {telemetry_url}")

        # Command PUSH socket
        self.cmd_socket = self.zmq_context.socket(zmq.PUSH)
        cmd_url = f"tcp://{self.jetson_ip}:{self.cmd_port}"
        self.cmd_socket.connect(cmd_url)
        self.get_logger().info(f"Connected to Jetson command at {cmd_url}")

        # Auto-ARM upon startup
        if self.auto_arm:
            self._send_arm(True)
            self.get_logger().info("Auto-ARM enabled: Sent ARM command to Jetson on startup.")

        # Worker thread for receiving telemetry
        self.running = True
        self.recv_thread = threading.Thread(target=self._telemetry_worker, daemon=True)
        self.recv_thread.start()

    def _send_arm(self, arm: bool):
        try:
            payload = {'arm': bool(arm)}
            packed = msgpack.packb(payload, use_bin_type=True)
            self.cmd_socket.send(packed, flags=zmq.NOBLOCK)
            self.is_armed = bool(arm)
        except Exception as e:
            self.get_logger().warn(f"Failed to send ARM command to Jetson: {e}")

    def _arm_motors_cb(self, request, response):
        self._send_arm(request.data)
        response.success = True
        response.message = f"Sent ARM={request.data} command to Jetson."
        self.get_logger().info(response.message)
        return response

    def _cmd_vel_cb(self, msg: Twist):
        try:
            # Auto-arm if not yet armed and user wants to drive
            if self.auto_arm and not self.is_armed and (abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001):
                self._send_arm(True)

            payload = {
                'linear_x': float(msg.linear.x),
                'angular_z': float(msg.angular.z)
            }
            packed = msgpack.packb(payload, use_bin_type=True)
            self.cmd_socket.send(packed, flags=zmq.NOBLOCK)
        except Exception as e:
            self.get_logger().warn(f"Failed to send cmd_vel to Jetson: {e}")

    def _telemetry_worker(self):
        while self.running and rclpy.ok():
            try:
                frames = self.sub_socket.recv_multipart()
                if len(frames) < 2:
                    continue
                topic, raw_payload = frames[0], frames[1]
                if topic not in (b'scan', b'odom', b'imu', b'battery', b'heartbeat'):
                    continue
                data = msgpack.unpackb(raw_payload, raw=False)

                if topic == b"scan":
                    self._handle_scan(data)
                elif topic == b"odom":
                    self._handle_odom(data)
                elif topic == b"imu":
                    self._handle_imu(data)
                elif topic == b"battery":
                    self._handle_battery(data)
                elif topic == b"heartbeat":
                    pass

            except zmq.Again:
                continue
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"Telemetry decode error: {e}")
                time.sleep(0.01)

    def _handle_scan(self, data: dict):
        """Converts raw points [(angle_deg, dist_mm, intensity), ...] to LaserScan."""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.laser_frame_id

        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2.0 * math.pi
        bins = 720
        scan_msg.angle_increment = (2.0 * math.pi) / bins
        duration = float(data.get('duration', 0.1))
        scan_msg.time_increment = duration / bins
        scan_msg.scan_time = duration
        scan_msg.range_min = 0.05
        scan_msg.range_max = 12.0

        ranges = [float('inf')] * bins
        intensities = [0.0] * bins

        points = data.get('points', [])
        for angle_deg, dist_mm, intensity in points:
            if dist_mm <= 0:
                continue
            dist_m = dist_mm / 1000.0
            if dist_m < scan_msg.range_min or dist_m > scan_msg.range_max:
                continue

            bin_idx = int((angle_deg % 360.0) * bins / 360.0) % bins
            # Keep nearer return
            if math.isinf(ranges[bin_idx]) or dist_m < ranges[bin_idx]:
                ranges[bin_idx] = dist_m
                intensities[bin_idx] = float(intensity)

        scan_msg.ranges = ranges
        scan_msg.intensities = intensities
        self.scan_pub.publish(scan_msg)

    def _handle_odom(self, data: dict):
        vx = data.get('vx', 0.0)
        wz = data.get('wz', 0.0)
        # twist.linear.x is the only field the EKF fuses from here.
        if not sensor_guard.odom_sample_ok(vx, wz,
                                           data.get('x', 0.0),
                                           data.get('y', 0.0),
                                           data.get('yaw', 0.0)):
            return self._odom_rejects.reject('odom payload outside physical range')

        odom_msg = Odometry()
        now_time = self.get_clock().now()
        stamp = now_time.to_msg()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id

        # Planar dead reckoning from wheel kinematics (Midpoint RK2)
        in_x = float(data.get('x', 0.0))
        in_y = float(data.get('y', 0.0))
        in_yaw = float(data.get('yaw', 0.0))

        if abs(in_x) > 1e-5 or abs(in_y) > 1e-5 or abs(in_yaw) > 1e-5:
            self.odom_x = in_x
            self.odom_y = in_y
            self.odom_yaw = in_yaw
        else:
            if self.last_odom_time is not None:
                dt = (now_time - self.last_odom_time).nanoseconds * 1e-9
                if 0.0 < dt < 0.5:
                    th_mid = self.odom_yaw + (float(wz) * dt * 0.5)
                    self.odom_x += float(vx) * math.cos(th_mid) * dt
                    self.odom_y += float(vx) * math.sin(th_mid) * dt
                    self.odom_yaw += float(wz) * dt
                    self.odom_yaw = math.atan2(math.sin(self.odom_yaw), math.cos(self.odom_yaw))
            self.last_odom_time = now_time

        odom_msg.pose.pose.position.x = float(self.odom_x)
        odom_msg.pose.pose.position.y = float(self.odom_y)
        odom_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(float(self.odom_yaw))
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.pose.covariance = _diag6(POSE_COV_DIAG)

        odom_msg.twist.twist.linear.x = float(data.get('vx', 0.0))
        odom_msg.twist.twist.linear.y = float(data.get('vy', 0.0))
        odom_msg.twist.twist.angular.z = float(data.get('wz', 0.0))
        odom_msg.twist.covariance = _diag6(TWIST_COV_DIAG)

        # The wheels are what license the ZUPT and the bias tracker.
        #
        # Use the per-wheel velocities the Jetson forwards rather than the vx/wz
        # it derived from them: those come straight off the STM32 as INTEGER
        # mm/s, so a stopped wheel is exactly 0 and "still" is exact. Testing
        # the derived floats instead would make standstill depend on rounding.
        if self.use_imu:
            left = data.get('left_vel_ms')
            right = data.get('right_vel_ms')
            if left is None or right is None:
                # Older Jetson bridge payloads carry only the derived velocities.
                moving = (abs(odom_msg.twist.twist.linear.x) > 1e-6
                          or abs(odom_msg.twist.twist.angular.z) > 1e-6)
            else:
                moving = (left != 0.0 or right != 0.0)
            self.gyro.set_wheel_motion(moving)

        self.odom_pub.publish(odom_msg)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = odom_msg.pose.pose.position.x
            t.transform.translation.y = odom_msg.pose.pose.position.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom_msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def _handle_imu(self, data: dict):
        if not self.use_imu:
            return
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame_id

        imu_msg.linear_acceleration.x = float(data.get('ax', 0.0))
        imu_msg.linear_acceleration.y = float(data.get('ay', 0.0))
        imu_msg.linear_acceleration.z = float(data.get('az', 9.81))

        # Bias, scale and ZUPT. Returns None while the initial bias calibration
        # is still running, and then NOTHING may be published: a biased vyaw in
        # those first seconds is exactly the error the EKF would integrate.
        if not sensor_guard.imu_sample_ok(
                data.get('ax', 0.0), data.get('ay', 0.0), data.get('az', 9.81),
                data.get('gx', 0.0), data.get('gy', 0.0), data.get('gz', 0.0)):
            return self._imu_rejects.reject('gyro/accel outside physical range')

        conditioned = self.gyro.condition(float(data.get('gz', 0.0)))
        if conditioned is None:
            return
        gz, yaw_var = conditioned

        imu_msg.angular_velocity.x = float(data.get('gx', 0.0))
        imu_msg.angular_velocity.y = float(data.get('gy', 0.0))
        imu_msg.angular_velocity.z = float(gz)

        # An all-zero covariance is not "unknown" to robot_localization - it
        # reads as "certain", which pins the yaw gain to 1.0 and lets a biased
        # gyro overrule the wheels outright.
        imu_msg.angular_velocity_covariance = self.gyro.angular_velocity_covariance(yaw_var)
        imu_msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04]

        # REP-145: a leading -1 declares that this message carries no absolute
        # orientation, which is true - there is no magnetometer.
        imu_msg.orientation_covariance = [-1.0] + [0.0] * 8
        self.imu_pub.publish(imu_msg)

    def _handle_battery(self, data: dict):
        v = Float32()
        v.data = float(data.get('voltage', 0.0))
        self.volt_pub.publish(v)

        c = Float32()
        c.data = float(data.get('current', 0.0))
        self.curr_pub.publish(c)

    def destroy_node(self):
        self.running = False
        self.sub_socket.close()
        self.cmd_socket.close()
        self.zmq_context.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JetsonBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
