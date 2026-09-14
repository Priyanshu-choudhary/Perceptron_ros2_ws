"""CLI: patrol start | status | cancel | dock.

`dock` prints the saved dock pose without needing the patrol node running,
which is the quickest way to answer "did the search actually record anything".
"""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from std_srvs.srv import Trigger

from perceptron_navigation import dock_store

TERMINAL = ('DOCKED', 'HOME', 'STOPPED', 'FAILED', 'IDLE')


class PatrolClient(Node):
    def __init__(self):
        super().__init__('patrol_client')
        self.status = None
        self.last_printed = None
        self.create_subscription(String, '/patrol/status', self._status, 10)

    def _status(self, msg):
        try:
            status = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        self.status = status
        battery = status.get('battery')
        text = f'{status["state"]}: {status["detail"]}'
        if battery is not None:
            text += f'  [battery {battery * 100:.0f}%'
            if status.get('battery_critical'):
                text += ', CRITICAL'
            elif status.get('battery_low'):
                text += ', low'
            text += ']'
        if text != self.last_printed:
            print(text, flush=True)
            self.last_printed = text

    def call(self, name):
        client = self.create_client(Trigger, name)
        if not client.wait_for_service(timeout_sec=10.0):
            print(f'{name} is not available; is the patrol node running '
                  '(patrol:=true)?')
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        result = future.result()
        if result is None:
            print(f'{name} did not respond')
            return False
        print(result.message)
        return result.success


def main(argv=None):
    parser = argparse.ArgumentParser(prog='patrol')
    parser.add_argument('command', choices=('start', 'status', 'cancel', 'dock'))
    parser.add_argument('--path', default=dock_store.DEFAULT_PATH,
                        help='dock store to read for the `dock` command')
    args = parser.parse_args(remove_ros_args(argv)[1:] if argv else None)

    if args.command == 'dock':
        record = dock_store.load(args.path)
        if record is None:
            print(f'No dock pose saved at {args.path}')
            return 2
        print(json.dumps(record, indent=2))
        return 0

    rclpy.init()
    node = PatrolClient()
    try:
        if args.command == 'cancel':
            return 0 if node.call('/patrol/cancel') else 1
        if args.command == 'start':
            if not node.call('/patrol/start'):
                return 1
        # Both start and status then follow the run until it settles.
        print('Ctrl-C stops watching; the patrol keeps running.')
        deadline = time.monotonic() + 3600.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            state = (node.status or {}).get('state')
            if args.command == 'status' and state is not None and node.last_printed:
                if state in TERMINAL:
                    break
            elif args.command == 'start' and state in TERMINAL and state != 'IDLE':
                break
        state = (node.status or {}).get('state')
        return 0 if state in ('DOCKED', 'HOME') else 2
    except KeyboardInterrupt:
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
