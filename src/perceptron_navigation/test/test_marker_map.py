"""Geometry tests for the wall-marker localiser.

The important one is test_round_trip_recovers_the_robot_pose: it builds a board
at a known map pose, puts the robot at a known map pose, renders where the
corners would land in the image, and checks the pipeline gets the robot back.
That exercises every frame convention at once -- the board axes, the optical
frame, and the inv() in T_map_base = T_map_board . inv(T_base_board) -- which
are the parts that are easy to get subtly, silently wrong.
"""

import math
import os
import tempfile

import cv2
import numpy as np
import pytest

from perceptron_navigation import marker_map as mm

CAMERA_MATRIX = np.array([
    [410.9212916385953, 0.0, 634.8042096493966],
    [0.0, 411.4395116912116, 347.58359871233904],
    [0.0, 0.0, 1.0]], dtype=np.float64)
DIST_COEFFS = np.array([0.030287865182229395, -0.002860707267771121,
                        -0.004223044189411946, -0.0003374289867825264,
                        -0.0003820853552867496], dtype=np.float64)

BOARD = {
    'tile_size': 0.06,
    'tile_spacing': 0.08,
    'ids': {0: 'top_left', 1: 'top_right', 2: 'bottom_left', 3: 'bottom_right'},
    'pose': {'x': 3.0, 'y': 1.0, 'z': 0.9, 'yaw_deg': 180.0},
}


def planar_transform(x, y, yaw, z=0.0):
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cos_y, -sin_y, 0.0],
                         [sin_y, cos_y, 0.0],
                         [0.0, 0.0, 1.0]], dtype=np.float64)
    return mm.make_transform(rotation, (x, y, z))


def base_from_camera_optical():
    """base_footprint <- camera_optical_link, as the URDF builds it.

    camera_joint puts camera_link at (0.18314, 0.01381, 0.0996) with no
    rotation; camera_optical_joint then applies rpy(-pi/2, 0, -pi/2) to reach
    the REP-103 optical frame (+Z out of the lens, +X right, +Y down).
    """
    half_pi = math.pi / 2.0
    rx = np.array([[1, 0, 0],
                   [0, math.cos(-half_pi), -math.sin(-half_pi)],
                   [0, math.sin(-half_pi), math.cos(-half_pi)]], dtype=np.float64)
    rz = np.array([[math.cos(-half_pi), -math.sin(-half_pi), 0],
                   [math.sin(-half_pi), math.cos(-half_pi), 0],
                   [0, 0, 1]], dtype=np.float64)
    return mm.make_transform(rz.dot(rx), (0.18314, 0.01381, 0.0996))


def render(board, map_from_base, base_from_camera):
    """Project every tile corner into the image. Returns (ids, corners)."""
    camera_from_board = (mm.invert_transform(base_from_camera)
                         .dot(mm.invert_transform(map_from_base))
                         .dot(mm.map_from_board(board)))
    rvec, _ = cv2.Rodrigues(camera_from_board[:3, :3])
    tvec = camera_from_board[:3, 3].reshape(3, 1)

    ids, corners = [], []
    for marker_id in sorted(board['ids']):
        points = np.array(mm.tile_object_points(board, marker_id),
                          dtype=np.float64).reshape(-1, 1, 3)
        projected, _ = cv2.projectPoints(points, rvec, tvec,
                                         CAMERA_MATRIX, DIST_COEFFS)
        ids.append(marker_id)
        corners.append([float(v) for v in projected.reshape(-1)])
    return ids, corners


def recover(board, ids, corners, base_from_camera):
    object_points, image_points, used = mm.assemble_correspondences(
        board, ids, corners)
    camera_from_board, error = mm.solve_board_pose(
        object_points, image_points, CAMERA_MATRIX, DIST_COEFFS)
    assert camera_from_board is not None, 'PnP failed'
    assert error < 1.0, 'reprojection error %.3f px on synthetic data' % error
    base_from_board = base_from_camera.dot(camera_from_board)
    return mm.robot_pose_in_map(mm.map_from_board(board), base_from_board), used


# --------------------------------------------------------------- round trip

@pytest.mark.parametrize('robot_x, robot_y, robot_yaw_deg', [
    (2.0, 1.0, 0.0),      # square on, one metre out
    (2.3, 1.0, 0.0),      # closer
    (2.1, 0.6, 25.0),     # off to one side, turned towards the board
    (2.1, 1.4, -25.0),    # and the other side
])
def test_round_trip_recovers_the_robot_pose(robot_x, robot_y, robot_yaw_deg):
    robot_yaw = math.radians(robot_yaw_deg)
    map_from_base = planar_transform(robot_x, robot_y, robot_yaw)
    base_from_camera = base_from_camera_optical()

    ids, corners = render(BOARD, map_from_base, base_from_camera)
    (x, y, yaw), used = recover(BOARD, ids, corners, base_from_camera)

    assert len(used) == 4
    assert x == pytest.approx(robot_x, abs=0.01)
    assert y == pytest.approx(robot_y, abs=0.01)
    assert yaw == pytest.approx(robot_yaw, abs=math.radians(1.0))


def test_round_trip_survives_losing_two_tiles():
    """Two tiles are the documented minimum; it should still land close."""
    map_from_base = planar_transform(2.2, 1.0, 0.0)
    base_from_camera = base_from_camera_optical()
    ids, corners = render(BOARD, map_from_base, base_from_camera)

    keep = [0, 3]  # opposite corners of the 2x2: the longest baseline left
    ids = [i for i in ids if i in keep]
    corners = [c for i, c in zip(sorted(BOARD['ids']), corners) if i in keep]

    (x, y, yaw), used = recover(BOARD, ids, corners, base_from_camera)
    assert len(used) == 2
    assert x == pytest.approx(2.2, abs=0.05)
    assert y == pytest.approx(1.0, abs=0.05)
    assert yaw == pytest.approx(0.0, abs=math.radians(5.0))


def test_corner_noise_stays_within_the_error_budget():
    """Quantify how corner noise propagates into the fix, at ~1 m.

    The dominant term is range error along the optical axis:

        sigma_Z  ~=  Z^2 . sigma_px / (fx . B)

    with B the board extent (tile_spacing + tile_size = 0.14 m). At Z ~= 0.8 m
    and a deliberately pessimistic 0.5 px of corner noise that predicts about
    0.9 cm, and the measured RMS below sits right on it. Sub-pixel refinement
    on a clean image is nearer 0.3 px, so this is a worst case, not a typical
    one. The bound is on RMS rather than the maximum because the maximum of a
    small sample is itself noisy -- that is what made this test flap at a 2 cm
    limit when the true sigma was 0.9 cm.

    This is the number that decides whether the whole approach is worth it:
    ~1 cm and well under a degree beats any human dragging a 2D Pose Estimate
    arrow in RViz, which is what it replaces.
    """
    rng = np.random.RandomState(0)
    map_from_base = planar_transform(2.0, 1.0, 0.0)
    base_from_camera = base_from_camera_optical()
    ids, clean = render(BOARD, map_from_base, base_from_camera)

    errors, yaw_errors = [], []
    for _ in range(50):
        noisy = [[v + rng.normal(0.0, 0.5) for v in quad] for quad in clean]
        (x, y, yaw), _ = recover(BOARD, ids, noisy, base_from_camera)
        errors.append(math.hypot(x - 2.0, y - 1.0))
        yaw_errors.append(abs(yaw))

    rms = math.sqrt(sum(e * e for e in errors) / len(errors))
    rms_yaw = math.sqrt(sum(e * e for e in yaw_errors) / len(yaw_errors))

    assert rms < 0.015, 'RMS position error %.4f m (expected ~0.009)' % rms
    assert max(errors) < 0.035, 'worst position error %.4f m' % max(errors)
    assert rms_yaw < math.radians(1.5),         'RMS yaw error %.2f deg' % math.degrees(rms_yaw)
    assert max(yaw_errors) < math.radians(4.0),         'worst yaw error %.2f deg' % math.degrees(max(yaw_errors))


# --------------------------------------------------------------- conventions

def test_wall_board_rotation_points_the_face_along_yaw():
    for yaw_deg in (0.0, 90.0, 180.0, -90.0, 37.0):
        yaw = math.radians(yaw_deg)
        rotation = mm.wall_board_rotation(yaw)
        # Column 2 is the board's +Z (face normal) expressed in the map.
        assert rotation[:, 2] == pytest.approx([math.cos(yaw), math.sin(yaw), 0.0])
        # Column 1 is the board's +Y (up the wall) -- straight up in the map.
        assert rotation[:, 1] == pytest.approx([0.0, 0.0, 1.0])
        assert np.linalg.det(rotation) == pytest.approx(1.0)


def test_tile_layout_matches_the_named_slots():
    spacing = BOARD['tile_spacing']
    centres = {}
    for marker_id in BOARD['ids']:
        points = np.array(mm.tile_object_points(BOARD, marker_id))
        centres[BOARD['ids'][marker_id]] = points.mean(axis=0)

    assert centres['top_left'] == pytest.approx([-spacing / 2, spacing / 2, 0.0])
    assert centres['top_right'] == pytest.approx([spacing / 2, spacing / 2, 0.0])
    assert centres['bottom_left'] == pytest.approx([-spacing / 2, -spacing / 2, 0.0])
    assert centres['bottom_right'] == pytest.approx([spacing / 2, -spacing / 2, 0.0])


def test_invert_transform_is_an_inverse():
    transform = planar_transform(1.5, -2.5, 0.7, z=0.3)
    assert transform.dot(mm.invert_transform(transform)) == pytest.approx(np.eye(4))


def test_intrinsics_scale_with_resolution():
    scaled = mm.scale_intrinsics(CAMERA_MATRIX, (1280, 720), (640, 360))
    assert scaled[0, 0] == pytest.approx(CAMERA_MATRIX[0, 0] / 2)
    assert scaled[0, 2] == pytest.approx(CAMERA_MATRIX[0, 2] / 2)
    assert scaled[1, 1] == pytest.approx(CAMERA_MATRIX[1, 1] / 2)
    assert scaled[1, 2] == pytest.approx(CAMERA_MATRIX[1, 2] / 2)
    unchanged = mm.scale_intrinsics(CAMERA_MATRIX, (1280, 720), (1280, 720))
    assert unchanged == pytest.approx(CAMERA_MATRIX)


# ------------------------------------------------------------------- gating

def test_poses_agree_rejects_a_disagreeing_window():
    good = [(1.0, 2.0, 0.10), (1.01, 2.0, 0.11), (0.99, 2.01, 0.09)]
    assert mm.poses_agree(good, 0.05, 0.05)
    bad = good + [(1.4, 2.0, 0.10)]
    assert not mm.poses_agree(bad, 0.05, 0.05)
    assert not mm.poses_agree([(1.0, 2.0, 0.0)], 0.05, 0.05)


def test_yaw_averaging_is_circular():
    """Poses either side of the pi discontinuity must not average to zero."""
    poses = [(0.0, 0.0, math.pi - 0.01), (0.0, 0.0, -math.pi + 0.01)]
    assert mm.poses_agree(poses, 0.05, 0.05)
    _, _, yaw = mm.mean_pose(poses)
    assert abs(abs(yaw) - math.pi) < 1e-6


def test_corners_are_central_rejects_the_frame_edge():
    middle = [(640, 360)] * 4
    assert mm.corners_are_central(middle, 1280, 720, 0.15)
    edge = [(640, 360), (640, 360), (1270, 360), (640, 360)]
    assert not mm.corners_are_central(edge, 1280, 720, 0.15)
    assert mm.corners_are_central(edge, 1280, 720, 0.0)


# ------------------------------------------------------------------ the YAML

def write_map(text):
    handle = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    handle.write(text)
    handle.close()
    return handle.name


VALID = """
frame_id: map
map_reference: room_map.yaml
boards:
  wall_north:
    tile_size: 0.06
    tile_spacing: 0.08
    ids: {0: top_left, 1: top_right, 2: bottom_left, 3: bottom_right}
    pose: {x: 2.31, y: -0.87, z: 0.9, yaw_deg: 180.0}
"""


def test_load_accepts_a_good_map():
    path = write_map(VALID)
    try:
        document = mm.load_marker_map(path)
        assert 'wall_north' in document['boards']
    finally:
        os.unlink(path)


@pytest.mark.parametrize('text, fragment', [
    (VALID.replace('tile_spacing: 0.08', 'tile_spacing: 0.02'), 'centre-to-centre'),
    (VALID.replace('1: top_right', '1: middle_left'), 'unknown slot'),
    (VALID.replace('boards:', 'markers:'), 'missing "boards"'),
    (VALID.replace('    pose: {x: 2.31, y: -0.87, z: 0.9, yaw_deg: 180.0}',
                   '    pose: {x: 2.31, z: 0.9, yaw_deg: 180.0}'), 'missing "y"'),
])
def test_load_rejects_a_bad_map(text, fragment):
    path = write_map(text)
    try:
        with pytest.raises(ValueError) as excinfo:
            mm.load_marker_map(path)
        assert fragment in str(excinfo.value)
    finally:
        os.unlink(path)


def test_load_rejects_an_id_claimed_by_two_boards():
    path = write_map(VALID + """
  wall_south:
    tile_size: 0.06
    tile_spacing: 0.08
    ids: {3: top_left}
    pose: {x: 0.0, y: 0.0, z: 0.9, yaw_deg: 0.0}
""")
    try:
        with pytest.raises(ValueError) as excinfo:
            mm.load_marker_map(path)
        assert 'claimed by both' in str(excinfo.value)
    finally:
        os.unlink(path)


def test_board_for_ids_picks_the_best_match_and_drops_strangers():
    path = write_map(VALID)
    try:
        document = mm.load_marker_map(path)
    finally:
        os.unlink(path)
    name, board, ids = mm.board_for_ids(document, [0, 2, 99])
    assert name == 'wall_north'
    assert ids == [0, 2]
    assert mm.board_for_ids(document, [99, 100])[0] is None
