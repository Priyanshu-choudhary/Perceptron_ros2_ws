#!/usr/bin/env python3
"""Bridge Nav2's velocity output onto the topic the base actually listens to.

Nav2 publishes `geometry_msgs/Twist` on /cmd_vel. The chain inside Humble's
nav2_bringup is controller_server -> /cmd_vel_nav -> velocity_smoother ->
/cmd_vel, because navigation_launch.py remaps `cmd_vel` to `cmd_vel_nav` on the
controller and `cmd_vel_smoothed` to `cmd_vel` on the smoother. The simulated
base listens on /diff_drive_controller/cmd_vel_unstamped because
diff_drive_controller has `use_stamped_vel: false`. The real base listens on
/cmd_vel via stm32_bridge_node.

A launch-file remap would also work, but a node is easier to reason about when
several things want to drive the robot: here you can see, in one place, whether
Nav2 or the docking controller is the one talking.

Publishing stops entirely while `enabled` is false, which is how the mission
node hands control over to the ArUco docking controller without the two of them
fighting over the base.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import SetBool


class CmdVelRelay(Node):

    def __init__(self):
        super().__init__('cmd_vel_relay')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/diff_drive_controller/cmd_vel_unstamped')
        self.declare_parameter('enabled', True)
        self.declare_parameter('warn_after_seconds', 30.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.enabled = bool(self.get_parameter('enabled').value)
        self.warn_after = float(self.get_parameter('warn_after_seconds').value)

        self.pub = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Twist, self.input_topic, self._cb, 10)
        self.create_service(SetBool, '/cmd_vel_relay/enable', self._srv_enable)

        self._seen = 0
        self.create_timer(self.warn_after, self._check_traffic)

        self.get_logger().info(
            f'Relaying {self.input_topic} -> {self.output_topic} (enabled={self.enabled})')

    def _cb(self, msg: Twist):
        self._seen += 1
        if self.enabled:
            self.pub.publish(msg)

    def _srv_enable(self, request, response):
        self.enabled = bool(request.data)
        if not self.enabled:
            self.pub.publish(Twist())      # never leave the base coasting
        response.success = True
        response.message = f'relay {"enabled" if self.enabled else "disabled"}'
        self.get_logger().info(response.message)
        return response

    def _check_traffic(self):
        if self._seen == 0:
            self.get_logger().warn(
                f'Nothing received on {self.input_topic} yet. If Nav2 is running it is '
                'publishing somewhere else. Find out where with '
                '"ros2 topic info /cmd_vel --verbose" and '
                '"ros2 topic info /cmd_vel_nav --verbose", then set input_topic to match.')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
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
