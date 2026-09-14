#!/usr/bin/env python3
"""Small CLI for driving the docking state machine.

    ros2 run perceptron_docking dock start     # begin docking, then follow the state
    ros2 run perceptron_docking dock cancel    # abort and stop the base
    ros2 run perceptron_docking dock status    # print state changes until Ctrl-C

Exactly equivalent to calling /docking/start and /docking/cancel by hand; this
just saves typing and prints the state transitions as they happen.
"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class DockClient(Node):

    def __init__(self):
        super().__init__('dock_client')
        self.last_state = None
        self.create_subscription(String, '/docking/status', self._status_cb, 10)

    def _status_cb(self, msg: String):
        if msg.data != self.last_state:
            self.last_state = msg.data
            print(f'  state: {msg.data}', flush=True)

    def call(self, service: str) -> bool:
        client = self.create_client(Trigger, service)
        if not client.wait_for_service(timeout_sec=5.0):
            print(f'{service} is not available - is docking_controller_node running?',
                  file=sys.stderr)
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None:
            print(f'{service} call timed out.', file=sys.stderr)
            return False
        print(f'{service}: {result.message}')
        return result.success


def usage() -> int:
    print(__doc__)
    return 2


def main(argv=None):
    argv = (argv if argv is not None else sys.argv)[1:]
    argv = [a for a in argv if not a.startswith('--ros-args') and a != '-r']
    command = argv[0] if argv else 'status'
    if command not in ('start', 'cancel', 'status'):
        return usage()

    rclpy.init()
    node = DockClient()
    try:
        if command == 'cancel':
            return 0 if node.call('/docking/cancel') else 1
        if command == 'start' and not node.call('/docking/start'):
            return 1

        print('Following /docking/status (Ctrl-C to stop watching; '
              'the robot keeps going)...')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if command == 'start' and node.last_state in ('DOCKED', 'FAILED'):
                break
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
