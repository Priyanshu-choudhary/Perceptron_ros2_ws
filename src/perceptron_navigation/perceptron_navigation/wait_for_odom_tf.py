"""Block until odom -> base_link is being published.

Nav2 brought up before the robot exists has no odom -> base_link transform,
so local_costmap spins on

    Invalid frame ID "odom" passed to canTransform argument target_frame

controller_server blocks part-way through its lifecycle activation, and
bt_navigator never activates. This is the barrier that stops that happening:
the launch file runs it between Gazebo and Nav2, and waits for it to exit.

The gate is the transform itself rather than a sensor topic. /scan appears the
moment Gazebo finishes spawning the model, but odom -> base_link only appears
once spawn_entity has exited, joint_state_broadcaster has been spawned and
diff_drive_controller after it - measurably later, and by a margin that varies
with machine load. Watching the transform also means not caring which sensors
are fitted: there is no /scan at all with lidar_type:=3d.
"""

import time

import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

TIMEOUT = 240.0
# Let the rest of the controller chain settle before releasing Nav2.
SETTLE = 3.0


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('wait_for_odom_tf')
    buffer = Buffer()
    TransformListener(buffer, node)

    start = time.monotonic()
    try:
        while time.monotonic() - start < TIMEOUT:
            rclpy.spin_once(node, timeout_sec=0.2)
            # Time() is "latest available", so this needs no clock of its own
            # and behaves the same under sim time.
            if buffer.can_transform('odom', 'base_link', Time()):
                node.get_logger().info(
                    'odom -> base_link live after %.0f s, starting Nav2'
                    % (time.monotonic() - start))
                time.sleep(SETTLE)
                return 0
        node.get_logger().error(
            'timed out after %.0f s waiting for odom -> base_link. The robot '
            'did not finish spawning: check that gzserver is running and that '
            'diff_drive_controller was spawned.' % TIMEOUT)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
