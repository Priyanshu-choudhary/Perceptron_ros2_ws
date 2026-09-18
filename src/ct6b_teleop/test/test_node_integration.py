#!/usr/bin/env python3
"""
Integration test for ct6b_teleop_node using virtual pseudo-terminal (pty).
"""

import os
import pty
import time
import rclpy
from geometry_msgs.msg import Twist

from ct6b_teleop.ct6b_teleop_node import CT6BTeleopNode


def make_packet(channels):
    pkt = bytearray([0x55, 0xFC])
    for ch in channels:
        pkt.append((ch >> 8) & 0xFF)
        pkt.append(ch & 0xFF)
    ch_sum = sum(pkt[2:16])
    pkt.append(ch_sum // 256)
    pkt.append(ch_sum % 256)
    return bytes(pkt)


def test_integration():
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)

    rclpy.init(args=['--ros-args', '-p', f'serial_port:={slave_name}', '-p', 'auto_detect_port:=false'])

    node = CT6BTeleopNode()
    received_twists = []

    sub_node = rclpy.create_node('test_sub_node')
    sub_node.create_subscription(Twist, '/cmd_vel', lambda msg: received_twists.append(msg), 10)

    try:
        # Wait for serial connection
        t_start = time.time()
        while time.time() - t_start < 2.0:
            if node.ser and node.ser.is_open:
                break
            time.sleep(0.05)

        assert node.ser and node.ser.is_open, "Failed to connect to virtual serial port"

        def pump_packets(pkt, duration=1.0):
            received_twists.clear()
            t_end = time.time() + duration
            while time.time() < t_end:
                os.write(master, pkt)
                rclpy.spin_once(node, timeout_sec=0.03)
                rclpy.spin_once(sub_node, timeout_sec=0.03)
                time.sleep(0.02)
            assert len(received_twists) > 0, "No twist messages received"
            return received_twists[-1]

        # 1. Neutral stick test
        neutral_pkt = make_packet([1500, 1500, 1000, 1500, 1000, 1000, 0])
        last_twist = pump_packets(neutral_pkt, duration=0.8)
        assert abs(last_twist.linear.x) < 0.001, f"Expected 0.0 linear, got {last_twist.linear.x}"
        assert abs(last_twist.angular.z) < 0.001, f"Expected 0.0 angular, got {last_twist.angular.z}"
        print(f"Neutral Test OK: lin={last_twist.linear.x}, ang={last_twist.angular.z}")

        # 2. Pitch stick forward test (with invert_pitch=True, Pitch=1000 produces +0.5 m/s)
        fwd_pkt = make_packet([1500, 1000, 1000, 1500, 1000, 1000, 0])
        last_twist = pump_packets(fwd_pkt, duration=0.8)
        assert abs(last_twist.linear.x - 0.5) < 0.02, f"Expected 0.5 linear, got {last_twist.linear.x}"
        assert abs(last_twist.angular.z) < 0.001, f"Expected 0.0 angular, got {last_twist.angular.z}"
        print(f"Forward (Stick Up) Test OK: lin={last_twist.linear.x}, ang={last_twist.angular.z}")

        # 3. Left Turn test (Roll=1000 produces +1.0 rad/s)
        left_pkt = make_packet([1000, 1500, 1000, 1500, 1000, 1000, 0])
        last_twist = pump_packets(left_pkt, duration=0.8)
        assert abs(last_twist.linear.x) < 0.001, f"Expected 0.0 linear, got {last_twist.linear.x}"
        assert abs(last_twist.angular.z - 1.0) < 0.02, f"Expected 1.0 angular, got {last_twist.angular.z}"
        print(f"Full Left Turn Test OK: lin={last_twist.linear.x}, ang={last_twist.angular.z}")

        print("\nALL INTEGRATION TESTS PASSED PERFECTLY!")
    finally:
        node.destroy_node()
        sub_node.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)


if __name__ == '__main__':
    test_integration()
