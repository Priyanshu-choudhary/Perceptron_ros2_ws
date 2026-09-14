"""Dock persistence and the patrol policy, without a running ROS graph."""

import json
import math

import pytest

from perceptron_navigation import dock_store
from perceptron_navigation.patrol_node import PatrolNode


# --- dock_store -------------------------------------------------------------

def test_staging_pose_sits_in_front_of_the_marker_facing_it():
    # Marker normal along -x, which is what the room_world dock gives: the
    # quaternion below is the one measured from a real FOUND result.
    quaternion = (0.5124, -0.4816, -0.4820, 0.5227)
    x, y, yaw = dock_store.staging_pose((6.529, 0.097), quaternion, 1.2)
    # Standing off along the normal means a smaller x, roughly the same y.
    assert x == pytest.approx(6.529 - 1.2, abs=0.05)
    assert y == pytest.approx(0.097, abs=0.15)
    # ...and looking back at the marker, i.e. towards +x.
    assert math.cos(yaw) > 0.99


def test_staging_distance_is_the_requested_standoff():
    quaternion = (0.0, 0.0, 0.0, 1.0)          # normal along +z: unusable
    with pytest.raises(ValueError):
        dock_store.staging_pose((0.0, 0.0), quaternion, 1.0)


def test_vertical_marker_normal_is_refused_rather_than_guessed():
    with pytest.raises(ValueError):
        dock_store.marker_normal((0.0, 0.0, 0.0, 1.0))


def test_save_then_load_round_trips(tmp_path):
    path = tmp_path / 'dock.json'
    record = {'marker_id': 42, 'frame': 'map',
              'position': {'x': 1.0, 'y': 2.0, 'z': 0.2},
              'orientation': {'x': 0.0, 'y': 0.7071, 'z': 0.0, 'w': 0.7071}}
    dock_store.save(str(path), record)
    assert dock_store.load(str(path))['marker_id'] == 42


def test_load_returns_none_for_missing_or_corrupt(tmp_path):
    assert dock_store.load(str(tmp_path / 'absent.json')) is None
    broken = tmp_path / 'broken.json'
    broken.write_text('{ not json', encoding='utf-8')
    assert dock_store.load(str(broken)) is None
    partial = tmp_path / 'partial.json'
    partial.write_text(json.dumps({'position': {'x': 1.0}}), encoding='utf-8')
    assert dock_store.load(str(partial)) is None


# --- patrol policy ----------------------------------------------------------

class Stub(PatrolNode):
    """A PatrolNode with every ROS interaction replaced by a record."""

    def __init__(self, dock=None, **params):
        self.calls = []
        self.goals = []
        self._params = {'search_marker_id': 42, 'dock_store_path': '/dev/null',
                        'dock_standoff': 1.2, 'navigation_timeout': 300.0,
                        'docking_timeout': 300.0, 'always_explore': False}
        self._params.update(params)
        self.state, self.detail = 'IDLE', ''
        self.dock = dock
        self.home = None
        self.marker_id = 42
        self.standoff = 1.2
        self.dock_path = '/dev/null'
        self.exploration_state = None
        self.docking_state = None
        self.low = self.critical = False
        self.percentage = 1.0
        self.goal_handle = None
        self.goal_kind = self.goal_result = None
        self.deadline = 1e9
        self.interrupted = False
        # The policy passes these to _call, so they must exist even though the
        # stubbed _call never looks at them.
        self.explore_start = self.explore_cancel = None
        self.dock_start = self.dock_cancel = None

    # the handful of node facilities the policy actually touches
    def get_parameter(self, name):
        return type('P', (), {'value': self._params[name]})()

    def get_logger(self):
        return type('L', (), {'info': lambda *a, **k: None,
                              'warn': lambda *a, **k: None,
                              'error': lambda *a, **k: None})()

    def now(self):
        return 100.0

    def _publish(self):
        pass

    def _call(self, client, what):
        self.calls.append(what)
        return True

    def _robot(self):
        return (0.0, 0.0, 0.0)

    def _navigate(self, x, y, yaw, kind):
        self.goals.append((kind, round(x, 3), round(y, 3)))
        self.goal_kind = kind
        return True

    def _abort_activity(self):
        self.calls.append('abort')


def dock_record():
    return {'marker_id': 42, 'frame': 'map',
            'position': {'x': 5.0, 'y': 0.0, 'z': 0.2},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            'staging': {'x': 3.8, 'y': 0.0, 'yaw': 0.0}}


def test_critical_battery_stops_everything_even_next_to_the_dock():
    node = Stub(dock=dock_record())
    node.state = 'GO_TO_DOCK'
    node.critical = True
    node._tick()
    assert node.state == 'STOPPED'
    assert 'abort' in node.calls


def test_low_battery_with_a_known_dock_goes_to_the_dock():
    node = Stub(dock=dock_record())
    node.state = 'EXPLORING'
    node.low = True
    node._tick()
    assert node.state == 'GO_TO_DOCK'


def test_low_battery_without_a_dock_returns_home():
    node = Stub(dock=None)
    node.state = 'EXPLORING'
    node.home = (0.0, 0.0, 0.0)
    node.low = True
    node._tick()
    assert node.state == 'RETURN_HOME'


def test_low_battery_interrupts_only_once():
    node = Stub(dock=None)
    node.state = 'EXPLORING'
    node.home = (0.0, 0.0, 0.0)
    node.low = True
    node._tick()
    node.state = 'EXPLORING'          # pretend something put it back
    node._tick()
    # The second tick must not re-abort: interrupted latches.
    assert node.calls.count('abort') == 1


def test_finding_the_marker_moves_on_to_the_dock():
    node = Stub(dock=None)
    node.state = 'EXPLORING'
    node.dock = dock_record()         # as _found would have set it
    node._tick()
    assert node.state == 'GO_TO_DOCK'


def test_exploration_ending_without_a_marker_returns_home():
    node = Stub(dock=None)
    node.state = 'EXPLORING'
    node.home = (0.0, 0.0, 0.0)
    node.exploration_state = 'NOT_FOUND'
    node._tick()
    assert node.state == 'RETURN_HOME'


def test_go_to_dock_sends_the_staging_pose_then_starts_docking():
    node = Stub(dock=dock_record())
    node.state = 'GO_TO_DOCK'
    node._tick()
    assert node.goals == [('dock', 3.8, 0.0)]
    node.goal_result = 'succeeded'
    node._tick()
    assert node.state == 'DOCKING'
    assert '/docking/start' in node.calls


def test_docked_is_terminal_and_docking_failure_goes_home():
    node = Stub(dock=dock_record())
    node.state = 'DOCKING'
    node.docking_state = 'DOCKED'
    node._tick()
    assert node.state == 'DOCKED'

    node = Stub(dock=dock_record())
    node.state = 'DOCKING'
    node.home = (0.0, 0.0, 0.0)
    node.docking_state = 'FAILED'
    node._tick()
    assert node.state == 'RETURN_HOME'


def test_a_known_dock_skips_exploration_unless_asked_otherwise():
    node = Stub(dock=dock_record())
    node.state = 'STARTING'
    node._tick()
    assert node.state == 'GO_TO_DOCK'

    node = Stub(dock=dock_record(), always_explore=True)
    node.state = 'STARTING'
    node._tick()
    assert node.state == 'EXPLORING'
    assert '/exploration/start' in node.calls
