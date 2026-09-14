"""Offline tests: no Gazebo, ROS graph, or robot commands are started."""

from concurrent.futures import Future
import importlib.util
import json
import math
from pathlib import Path
from unittest.mock import Mock

import cv2
import numpy as np
import pytest
import yaml
from launch import LaunchContext
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker, MarkerArray

from perceptron_navigation.aruco_search_detector import dictionary_for, observations
from perceptron_navigation.exploration_node import ExplorationNode
from perceptron_navigation.exploration_storage import save_run
from perceptron_navigation.frontier_planner import Grid, choose_target, reachable_distances


PACKAGE = Path(__file__).resolve().parents[1]


def partial_room():
    data = np.full((60, 80), 100, dtype=np.int16)
    data[10:50, 5:35] = 0
    data[10:50, 35:75] = -1
    return Grid(data, 0.1)


def test_frontier_goal_stays_in_free_space_with_clearance():
    grid = partial_room()
    result = choose_target(grid, (1.5, 3.0))
    assert result.target is not None
    assert result.target.kind == 'frontier'
    row, col = grid.cell(result.target.x, result.target.y)
    assert grid.data[row, col] == 0
    assert result.target.x <= 3.15  # footprint does not extend into unseen space
    assert result.target.x > 2.5
    assert abs(result.target.yaw) < 0.2


def test_disconnected_room_is_not_a_reachable_frontier():
    grid = partial_room()
    grid.data[:, 30] = 100
    result = choose_target(grid, (1.5, 3.0))
    assert result.target is None
    assert result.frontier_cells > 0


def test_narrow_door_is_rejected_but_wide_door_can_be_crossed():
    grid = partial_room()
    grid.data[10:50, 5:55] = 0
    grid.data[10:50, 55:75] = -1
    grid.data[:, 30] = 100
    grid.data[28:32, 30] = 0  # 40 cm < robot diameter
    assert choose_target(grid, (1.5, 3.0)).target is None
    grid.data[23:37, 30] = 0
    target = choose_target(grid, (1.5, 3.0)).target
    assert target is not None and target.x > 4.5


def test_no_diagonal_corner_cutting():
    distances = reachable_distances(np.eye(3, dtype=bool), (0, 0))
    assert distances[0, 0] == 0
    assert distances[1, 1] == -1


def test_map_origin_rotation_roundtrip():
    grid = Grid(np.zeros((20, 30)), 0.1, -4.0, 2.0, math.pi / 2)
    for cell in ((0, 0), (5, 6), (19, 29)):
        assert grid.cell(*grid.world(*cell)) == cell


def test_frontier_world_coordinates_follow_rotated_map_origin():
    grid = partial_room()
    original = choose_target(grid, (1.5, 3.0)).target
    grid.origin_x, grid.origin_y, grid.origin_yaw = -3, 2, math.pi / 2
    rotated = choose_target(grid, (-6.0, 3.5)).target
    assert rotated.x == pytest.approx(-3 - original.y)
    assert rotated.y == pytest.approx(2 + original.x)


def test_search_inspects_known_room_when_no_frontiers_remain():
    data = np.full((60, 80), 100)
    data[5:55, 5:75] = 0
    grid = Grid(data, 0.1)
    assert choose_target(grid, (1.5, 3.0)).target is None
    target = choose_target(grid, (1.5, 3.0), search=True, scanned=[(1.5, 3.0)]).target
    assert target is not None and target.kind == 'viewpoint'
    assert math.hypot(target.x - 1.5, target.y - 3.0) >= 1.2


def test_exclusions_prevent_repeated_failed_goals():
    grid = partial_room()
    first = choose_target(grid, (1.5, 3.0)).target
    second = choose_target(grid, (1.5, 3.0), excluded=[(first.x, first.y, 0.8)]).target
    assert second is not None
    assert math.hypot(second.x - first.x, second.y - first.y) > 0.8


def test_map_expansion_changes_the_next_exploration_goal():
    grid = partial_room()
    first = choose_target(grid, (1.5, 3.0)).target
    grid.data[10:50, 35:55] = 0
    second = choose_target(grid, (first.x, first.y)).target
    assert second is not None and second.x > first.x + 1.0


def test_save_map_flips_rows_and_keeps_unknown(tmp_path):
    grid = Grid(np.array([[0, -1], [100, 0]]), 0.1, -1, 2, 0.3)
    path = Path(save_run(tmp_path, 'run', grid, {'found_marker': {'id': 10}}))
    assert json.loads(path.read_text())['found_marker']['id'] == 10
    metadata = yaml.safe_load(path.with_name('map.yaml').read_text())
    assert metadata['origin'] == [-1, 2, 0.3]
    assert path.with_name('map.pgm').read_bytes().endswith(bytes([0, 254, 254, 205]))


def test_detector_reports_multiple_ids_without_changing_docking_target():
    dictionary = dictionary_for('DICT_5X5_250')
    image = np.full((600, 800), 255, dtype=np.uint8)
    for marker_id, x in ((10, 140), (42, 480)):
        image[220:340, x:x + 120] = cv2.aruco.drawMarker(dictionary, marker_id, 120)
    matrix = np.array([[476.7, 0, 400], [0, 476.7, 300], [0, 0, 1]], dtype=float)
    found = observations(image, matrix, np.zeros(5), dictionary, 0.15)
    assert {item[0] for item in found} == {10, 42}
    assert all(0.5 < item[1][2] < 0.7 for item in found)


def test_wrong_dictionary_is_rejected():
    with pytest.raises(ValueError):
        dictionary_for('not_a_dictionary')


def controller_stub():
    node = object.__new__(ExplorationNode)
    node.state = 'NAVIGATING'
    node.pending_terminal = None
    node.cancel_started_wall = None
    node.motion = None
    node._state = lambda state, detail: setattr(node, 'state', state)
    node._finish = Mock()
    node.get_logger = Mock(return_value=Mock())
    return node


def test_cancel_before_goal_acceptance_cancels_late_handle():
    node = controller_stub()
    slot = {'kind': 'navigate', 'handle': None, 'cancel_sent': False}
    node.motion = slot
    node._stop('CANCELLED', 'test')
    assert node.state == 'CANCELLING'
    node._finish.assert_not_called()
    handle = Mock(accepted=True)
    future = Future()
    future.set_result(handle)
    node._accepted(future, slot)
    handle.cancel_goal_async.assert_called_once()
    node._finish.assert_not_called()
    node._motion_done(slot, False)
    node._finish.assert_called_once_with('CANCELLED', 'test')


def test_cancel_does_not_send_duplicate_requests():
    node = controller_stub()
    handle = Mock()
    node.motion = {'handle': handle, 'cancel_sent': False}
    node._cancel_motion()
    node._cancel_motion()
    handle.cancel_goal_async.assert_called_once()


def test_superseded_goal_acceptance_cannot_start_motion_untracked():
    node = controller_stub()
    handle = Mock(accepted=True)
    future = Future()
    future.set_result(handle)
    node._accepted(future, {'old': 'slot'})
    handle.cancel_goal_async.assert_called_once()


def test_search_requires_requested_id_and_distinct_fresh_observations(monkeypatch):
    node = controller_stub()
    node.grid = partial_room()
    node.target_id = 10
    node.started = 5.0
    node.now = lambda: 10.0
    settings = {'dictionary_name': 'DICT_5X5_250', 'map_frame': 'map',
                'sensor_timeout': 3.0, 'marker_consistency_distance': 0.3,
                'marker_confirmations': 3, 'marker_size': 0.15,
                'marker_transform_timeout': 0.15}
    node.p = settings.__getitem__
    node.confirmation = None
    node.found_pub = Mock()
    node.tf_buffer = Mock()
    node._stop = Mock()
    node.found = None
    monkeypatch.setattr('perceptron_navigation.exploration_node.do_transform_pose_stamped',
                        lambda pose, tf: pose)
    marker = Marker()
    marker.ns = 'DICT_5X5_250'
    marker.id = 42
    marker.header.frame_id = 'camera_optical_link'
    marker.header.stamp.sec = 9
    marker.pose.position.x = 2.0
    marker.pose.orientation.w = 1.0
    node._markers(MarkerArray(markers=[marker]))
    assert node.confirmation is None
    marker.id = 10
    marker.header.stamp.sec = 1  # stale
    node._markers(MarkerArray(markers=[marker]))
    assert node.confirmation is None
    marker.header.stamp.sec = 9
    node._markers(MarkerArray(markers=[marker]))
    node._markers(MarkerArray(markers=[marker]))  # repeated stamp must not count
    assert node.confirmation[2] == 1
    marker.header.stamp.nanosec = 200000000
    node._markers(MarkerArray(markers=[marker]))
    node._stop.assert_not_called()
    marker.header.stamp.nanosec = 400000000
    node._markers(MarkerArray(markers=[marker]))
    assert node.found['id'] == 10
    assert node.found['frame'] == 'map'
    node._stop.assert_called_once()


def test_target_change_is_rejected_while_exploring():
    node = controller_stub()
    node.dictionary_size = 250
    assert not node._validate_parameters([Parameter('target_marker_id', value=11)]).successful
    node.state = 'IDLE'
    assert node._validate_parameters([Parameter('target_marker_id', value=11)]).successful
    assert not node._validate_parameters([Parameter('target_marker_id', value=250)]).successful


def launch_module(filename):
    spec = importlib.util.spec_from_file_location('test_launch', PACKAGE / 'launch' / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('overrides', [
    {'slam': 'false'}, {'lidar_type': '3d'}, {'global_ekf': 'true'},
    {'localization': 'amcl'}, {'nav_profile': 'invalid'},
])
def test_invalid_exploration_launch_combinations_fail_before_launch(overrides):
    context = LaunchContext()
    context.launch_configurations.update(dict(exploration='true', slam='true',
                                              lidar_type='2d', global_ekf='false',
                                              localization='auto', nav_profile='dwb'))
    context.launch_configurations.update(overrides)
    with pytest.raises(RuntimeError):
        launch_module('nav_simulation.launch.py')._validate_exploration(context)


def test_valid_exploration_launch_configuration():
    context = LaunchContext()
    context.launch_configurations.update(dict(exploration='true', slam='true',
                                              lidar_type='both', global_ekf='false',
                                              localization='auto', nav_profile='mppi'))
    assert launch_module('nav_simulation.launch.py')._validate_exploration(context) == []
