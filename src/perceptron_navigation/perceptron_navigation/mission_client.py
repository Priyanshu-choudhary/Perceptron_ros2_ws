#!/usr/bin/env python3
"""CLI for the patrol mission.

    ros2 run perceptron_navigation mission start    # run the route, follow the state
    ros2 run perceptron_navigation mission cancel   # abort and stop
    ros2 run perceptron_navigation mission status   # watch until Ctrl-C

Equivalent to calling /mission/start and /mission/cancel by hand; this just
prints the state transitions as they happen.
"""

import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MissionClient(Node):

    def __init__(self):
        super().__init__('mission_client')
        self.last = None
        self.create_subscription(String, '/mission/status', self._cb, 10)

    def _cb(self, msg: String):
        if msg.data != self.last:
            self.last = msg.data
            print(f'  {msg.data}', flush=True)

    def call(self, service: str) -> bool:
        client = self.create_client(Trigger, service)
        if not client.wait_for_service(timeout_sec=5.0):
            print(f'{service} is not available - is mission_node running?', file=sys.stderr)
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None:
            print(f'{service} call timed out.', file=sys.stderr)
            return False
        print(f'{service}: {result.message}')
        return result.success


def main(argv=None):
    argv = (argv if argv is not None else sys.argv)[1:]
    argv = [a for a in argv if not a.startswith('--ros-args') and a != '-r']
    command = argv[0] if argv else 'status'
    if command not in ('start', 'cancel', 'status'):
        print(__doc__)
        return 2

    rclpy.init()
    node = MissionClient()
    try:
        if command == 'cancel':
            return 0 if node.call('/mission/cancel') else 1
        if command == 'start' and not node.call('/mission/start'):
            return 1

        print('Following /mission/status (Ctrl-C to stop watching; '
              'the robot keeps going)...')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if command == 'start' and node.last and node.last.split(' |')[0] in (
                    'DONE', 'FAILED', 'CANCELLED'):
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
