#!/usr/bin/env python3
"""
ROS 2 Teleop Node for FlySky FS-CT6B RC Transmitter.

Translates Pitch (forward/backward) and Roll (left/right steering) into
geometry_msgs/msg/Twist on /cmd_vel at a fixed rate (default: 10 Hz).
Includes deadzone centering, failsafe watchdog, and serial auto-reconnect.
"""

import glob
import os
import sys
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class CT6BTeleopNode(Node):
    """ROS 2 Node translating FlySky CT6B serial data to /cmd_vel."""

    def __init__(self):
        super().__init__('ct6b_teleop_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('auto_detect_port', True)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('channel_pitch', 1)  # Index 1 = Ch2 Pitch (Elevator)
        self.declare_parameter('channel_roll', 0)   # Index 0 = Ch1 Roll (Aileron)
        self.declare_parameter('center_pwm', 1500)
        self.declare_parameter('min_pwm', 1000)
        self.declare_parameter('max_pwm', 2000)
        self.declare_parameter('deadzone', 40)
        self.declare_parameter('invert_pitch', True)
        self.declare_parameter('invert_roll', True)
        self.declare_parameter('watchdog_timeout', 0.5)

        # Retrieve parameters
        self.port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.auto_detect = self.get_parameter('auto_detect_port').get_parameter_value().bool_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.max_lin = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_ang = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.ch_pitch = self.get_parameter('channel_pitch').get_parameter_value().integer_value
        self.ch_roll = self.get_parameter('channel_roll').get_parameter_value().integer_value
        self.center_pwm = self.get_parameter('center_pwm').get_parameter_value().integer_value
        self.min_pwm = self.get_parameter('min_pwm').get_parameter_value().integer_value
        self.max_pwm = self.get_parameter('max_pwm').get_parameter_value().integer_value
        self.deadzone = self.get_parameter('deadzone').get_parameter_value().integer_value

        # Handle boolean or string inputs (e.g. from launch LaunchConfiguration strings)
        raw_pitch = self.get_parameter('invert_pitch').value
        self.invert_pitch = (str(raw_pitch).lower() in ('true', '1', 'yes', 'on')) if not isinstance(raw_pitch, bool) else raw_pitch

        raw_roll = self.get_parameter('invert_roll').value
        self.invert_roll = (str(raw_roll).lower() in ('true', '1', 'yes', 'on')) if not isinstance(raw_roll, bool) else raw_roll

        self.watchdog_sec = self.get_parameter('watchdog_timeout').get_parameter_value().double_value

        # Publisher
        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # State tracking
        self.ser: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        self.running = True
        self.last_packet_time = 0.0
        self.last_valid_channels: Optional[List[int]] = None
        self.packets_received = 0
        self.last_log_time = 0.0

        # Current command values
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        # Log configuration
        self.get_logger().info('====================================================')
        self.get_logger().info('FlySky CT6B ROS 2 Teleop Node Starting...')
        self.get_logger().info(f'Target Topic       : {self.cmd_vel_topic}')
        self.get_logger().info(f'Publish Rate       : {self.rate_hz} Hz')
        self.get_logger().info(f'Max Linear Speed   : {self.max_lin} m/s')
        self.get_logger().info(f'Max Angular Speed  : {self.max_ang} rad/s')
        self.get_logger().info(f'Deadzone           : +/- {self.deadzone} us around {self.center_pwm} us')
        self.get_logger().info(f'Invert Pitch       : {self.invert_pitch}')
        self.get_logger().info(f'Invert Roll        : {self.invert_roll}')
        self.get_logger().info(f'Configured Port    : {self.port} @ {self.baud} baud')
        self.get_logger().info('====================================================')

        # Start serial worker thread
        self.serial_thread = threading.Thread(target=self._serial_worker, daemon=True)
        self.serial_thread.start()

        # Start 10 Hz ROS timer
        timer_period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(timer_period, self._timer_callback)

    def _find_available_port(self) -> Optional[str]:
        """Auto-detect available serial ports if the configured port is missing."""
        candidates = [self.port]
        if sys.platform.startswith('win'):
            candidates.extend([f'COM{i}' for i in range(1, 20)])
        else:
            candidates.extend(sorted(glob.glob('/dev/ttyUSB*')))
            candidates.extend(sorted(glob.glob('/dev/ttyACM*')))
            candidates.extend(sorted(glob.glob('/dev/serial/by-id/*')))

        # Remove duplicates while preserving order
        seen = set()
        unique_candidates = []
        for c in candidates:
            if c and c not in seen:
                seen.add(c)
                unique_candidates.append(c)

        for p in unique_candidates:
            if sys.platform.startswith('win'):
                return p
            elif os.path.exists(p):
                return p
        return None

    def _open_serial(self) -> bool:
        """Attempt to open the serial port."""
        port_to_try = self.port
        if not os.path.exists(port_to_try) and not port_to_try.startswith('COM') and self.auto_detect:
            detected = self._find_available_port()
            if detected:
                self.get_logger().info(f'Configured port {self.port} not found. Found alternative: {detected}')
                port_to_try = detected

        try:
            self.ser = serial.Serial(
                port=port_to_try,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.get_logger().info(f'Connected to CT6B on {port_to_try} at {self.baud} baud.')
            return True
        except serial.SerialException as e:
            self.get_logger().warn(f'Could not open serial port {port_to_try}: {e}')
            return False
        except Exception as e:
            self.get_logger().error(f'Unexpected error opening {port_to_try}: {e}')
            return False

    def _parse_packet(self, packet: bytearray) -> Optional[List[int]]:
        """
        Parse single 18-byte FS-CT6B packet:
        Header: 0x55 0xFC
        7 Channels: 2 bytes each (big-endian)
        Checksum: Sum of bytes 2-15
        """
        if len(packet) < 18 or packet[0] != 0x55 or packet[1] != 0xFC:
            return None

        channels = []
        for j in range(7):
            high = packet[2 + 2 * j]
            low = packet[3 + 2 * j]
            channels.append((high << 8) | low)

        # Checksum calculation (Bytes 2-15)
        ch_sum = sum(packet[2:16])
        expected_high = ch_sum // 256
        expected_low = ch_sum % 256

        actual_high = packet[16]
        actual_low = packet[17]

        if expected_high == actual_high and expected_low == actual_low:
            return channels
        return None

    def _map_axis(self, raw_val: int, invert: bool) -> float:
        """
        Maps raw PWM pulse (1000-2000 us) to normalized range [-1.0, 1.0] with deadband.
        Returns 0.0 within [center - deadzone, center + deadzone].
        """
        delta = raw_val - self.center_pwm
        if abs(delta) <= self.deadzone:
            return 0.0

        if delta > 0:
            span = self.max_pwm - (self.center_pwm + self.deadzone)
            norm = (delta - self.deadzone) / span if span > 0 else 1.0
            norm = min(1.0, max(0.0, norm))
        else:
            span = (self.center_pwm - self.deadzone) - self.min_pwm
            norm = (delta + self.deadzone) / span if span > 0 else -1.0
            norm = max(-1.0, min(0.0, norm))

        return -norm if invert else norm

    def _serial_worker(self):
        """Background thread to read and parse CT6B serial stream."""
        buf = bytearray()

        while self.running:
            if not self.ser or not self.ser.is_open:
                if not self._open_serial():
                    time.sleep(1.5)
                    continue

            try:
                n = self.ser.in_waiting
                if n > 0:
                    data = self.ser.read(n)
                    buf.extend(data)

                # Process all complete 18-byte packets in the buffer
                while True:
                    idx = buf.find(b'\x55\xFC')
                    if idx == -1 or idx + 18 > len(buf):
                        if idx != -1:
                            del buf[:idx]
                        break

                    pkt = buf[idx:idx + 18]
                    channels = self._parse_packet(pkt)

                    if channels is not None:
                        now = time.time()
                        with self.serial_lock:
                            self.last_packet_time = now
                            self.last_valid_channels = channels
                            self.packets_received += 1

                            # Pitch (Elevator): Channel 2 (index 1) -> Forward/Backward linear.x
                            pitch_raw = channels[self.ch_pitch]
                            norm_pitch = self._map_axis(pitch_raw, self.invert_pitch)
                            self.current_linear_x = norm_pitch * self.max_lin

                            # Roll (Aileron): Channel 1 (index 0) -> Left/Right angular.z
                            # In standard Mode 2, stick LEFT has PWM < center (<1500).
                            # Left turn in ROS is POSITIVE angular.z.
                            # Therefore default mapping inverts the stick direction.
                            roll_raw = channels[self.ch_roll]
                            norm_roll = self._map_axis(roll_raw, not self.invert_roll)
                            self.current_angular_z = norm_roll * self.max_ang

                    # Discard processed chunk
                    del buf[:idx + 18]

                time.sleep(0.005)

            except serial.SerialException as e:
                self.get_logger().warn(f'Serial connection lost: {e}. Reconnecting...')
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = None
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().error(f'Unexpected error in serial worker: {e}')
                time.sleep(1.0)

        # Clean close on shutdown
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _timer_callback(self):
        """Fixed-rate timer callback (10 Hz) to publish Twist on cmd_vel."""
        now = time.time()
        twist = Twist()

        with self.serial_lock:
            dt = now - self.last_packet_time
            is_alive = (self.last_packet_time > 0.0) and (dt < self.watchdog_sec)

            if is_alive:
                twist.linear.x = float(self.current_linear_x)
                twist.angular.z = float(self.current_angular_z)
                status_str = 'ACTIVE'
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                status_str = f'TIMEOUT ({dt:.1f}s ago)' if self.last_packet_time > 0 else 'NO DATA'

            ch_info = self.last_valid_channels

        # Publish Twist message
        self.pub_cmd_vel.publish(twist)

        # Periodic operator terminal logging (~1 Hz)
        if now - self.last_log_time >= 1.0:
            self.last_log_time = now
            if ch_info:
                p_raw = ch_info[self.ch_pitch]
                r_raw = ch_info[self.ch_roll]
                self.get_logger().info(
                    f'[{status_str}] Pitch: {p_raw} (lin: {twist.linear.x:+.2f} m/s) | '
                    f'Roll: {r_raw} (ang: {twist.angular.z:+.2f} rad/s) | Pkts: {self.packets_received}'
                )
            else:
                self.get_logger().info(f'[{status_str}] Waiting for CT6B transmitter packets on {self.port}...')

    def destroy_node(self):
        """Clean shutdown: send zero-velocity stop message before closing."""
        self.running = False
        try:
            stop_twist = Twist()
            stop_twist.linear.x = 0.0
            stop_twist.angular.z = 0.0
            self.pub_cmd_vel.publish(stop_twist)
            self.get_logger().info('Sent zero-velocity stop to /cmd_vel.')
        except Exception:
            pass

        if self.ser and self.ser.is_open:
            self.ser.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CT6BTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('CT6B teleop stopped by keyboard interrupt.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
