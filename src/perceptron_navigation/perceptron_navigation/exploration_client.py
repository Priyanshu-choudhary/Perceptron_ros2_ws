"""CLI: explore start | search ID | cancel | status."""

import argparse
import json
import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from std_srvs.srv import Trigger


class ExplorationClient(Node):
    def __init__(self):
        super().__init__('exploration_client')
        self.status = None
        self.last_printed = None
        self.run_id = None
        self.create_subscription(String, '/exploration/status', self._status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _status(self, msg):
        try:
            status = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if self.run_id is not None and status.get('run_id') != self.run_id:
            return
        self.status = status
        text = f'{status["state"]}: {status["detail"]}'
        if status.get('result_file'):
            text += '\n  Saved: ' + status['result_file']
        if text != self.last_printed:
            print(text, flush=True)
            self.last_printed = text

    def call(self, service_type, name, request):
        client = self.create_client(service_type, name)
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(name + ' unavailable; launch with exploration:=true')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            if not future.done():
                raise RuntimeError(name + ' timed out; inspect explore status before retrying')
            return future.result()
        finally:
            self.destroy_client(client)

    def begin(self, marker_id):
        request = SetParametersAtomically.Request(parameters=[Parameter(
            name='target_marker_id', value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=marker_id))])
        result = self.call(SetParametersAtomically,
                           '/exploration_node/set_parameters_atomically', request)
        if not result.result.successful:
            raise RuntimeError(result.result.reason)
        response = self.call(Trigger, '/exploration/start', Trigger.Request())
        if not response.success:
            raise RuntimeError(response.message)
        self.run_id = json.loads(response.message)['run_id']
        self.status = None


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('start', help='Explore and map reachable space')
    search = commands.add_parser('search', help='Explore and search for one ID in the configured dictionary')
    search.add_argument('marker_id', type=int)
    commands.add_parser('cancel', help='Cancel the active exploration/search')
    commands.add_parser('status', help='Watch status without controlling the robot')
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    if options.command == 'search' and options.marker_id < 0:
        parser.error('marker_id must be nonnegative')
    # Keep the ROS context alive while Ctrl-C sends a cancellation request.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ExplorationClient()
    started = False
    try:
        if options.command == 'cancel':
            response = node.call(Trigger, '/exploration/cancel', Trigger.Request())
            print(response.message)
            return 0 if response.success else 1
        if options.command in ('start', 'search'):
            node.begin(options.marker_id if options.command == 'search' else -1)
            started = True
            print('Ctrl-C requests cancellation of this run. Use explore status to watch separately.')
        finished_at = None
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if started and node.status:
                state = node.status['state']
                if state in ('FOUND', 'COMPLETE', 'NOT_FOUND', 'EXHAUSTED',
                             'FAILED', 'CANCELLED', 'STOP_UNCONFIRMED'):
                    # The terminal state precedes the saved-file status update.
                    finished_at = finished_at or time.monotonic()
                    if node.status.get('result_file') or time.monotonic() - finished_at > 2.0:
                        return 0 if state in ('FOUND', 'COMPLETE') else 2
        return 1
    except KeyboardInterrupt:
        if started and rclpy.ok():
            try:
                print(node.call(Trigger, '/exploration/cancel', Trigger.Request()).message)
            except RuntimeError as exc:
                print('Cancellation unconfirmed: ' + str(exc))
        return 130
    except RuntimeError as exc:
        print(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
