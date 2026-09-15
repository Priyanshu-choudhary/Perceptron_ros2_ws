"""Wall-marker map: board geometry, PnP, and the map-frame pose composition.

Deliberately free of ROS imports so the geometry can be tested without a
running graph -- see test/test_marker_map.py.

THE WHOLE IDEA, IN ONE LINE

    T_map_base = T_map_board  .  inv(T_base_board)

`T_base_board` is measured (PnP on the corner pixels the Jetson sent, then
through the URDF's base_footprint -> camera_optical_link). `T_map_board` is
surveyed once and stored in marker_map.yaml. Everything else here is
bookkeeping around those two.

FRAME CONVENTIONS, WHICH ARE THE EASY THING TO GET WRONG

  marker/board frame   +X right, +Y up, +Z out of the printed face
                       (matches the corner model that pairs with OpenCV's
                       detectMarkers order: TL, TR, BR, BL)
  camera_optical_link  +X right, +Y down, +Z out of the lens   (REP 103)
  map / base_footprint +X forward, +Y left, +Z up              (REP 103)

A board hanging flat on a wall has its +Y along map +Z, and its +Z pointing
out into the room. `yaw_deg` in the YAML is the map-frame direction that +Z
points -- so a board you read while facing east (yaw 0) has yaw_deg 180.
"""

import math

import cv2
import numpy as np
import yaml

# A board pose is meaningless unless `map` means the same thing it meant when
# the board was surveyed, which is why the YAML carries map_reference. Same
# reasoning as dock_store.py.
REQUIRED_TOP_LEVEL = ('frame_id', 'boards')

# OpenCV corner order within one tile, in that tile's own plane, as multiples
# of half the tile side: top-left, top-right, bottom-right, bottom-left.
_TILE_CORNER_SIGNS = ((-1.0, 1.0), (1.0, 1.0), (1.0, -1.0), (-1.0, -1.0))

# Where each named slot sits in the 2x2, in units of the centre-to-centre
# spacing, on the same +X right / +Y up axes.
_SLOT_OFFSETS = {
    'top_left': (-0.5, 0.5),
    'top_right': (0.5, 0.5),
    'bottom_left': (-0.5, -0.5),
    'bottom_right': (0.5, -0.5),
    'centre': (0.0, 0.0),
    'center': (0.0, 0.0),
}


# ---------------------------------------------------------------- transforms

def rotation_to_yaw(rotation):
    """Planar yaw of a 3x3 rotation, in radians."""
    return math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))


def yaw_to_quaternion(yaw):
    """(x, y, z, w) for a rotation of `yaw` about +Z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def make_transform(rotation, translation):
    """4x4 homogeneous transform from a 3x3 rotation and a 3-vector."""
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


def invert_transform(transform):
    """Inverse of a rigid 4x4. Transpose the rotation; do not call inv()."""
    rotation = transform[:3, :3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T.dot(transform[:3, 3])
    return inverse


def wall_board_rotation(yaw):
    """map <- board rotation for a board hanging flat and upright on a wall.

    Board +Z (out of the face) points along `yaw` in the map plane, and board
    +Y (up the wall) points along map +Z. The remaining axis follows from
    right-handedness: X = Y x Z.
    """
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    return np.array([
        [-sin_y, 0.0, cos_y],
        [cos_y, 0.0, sin_y],
        [0.0, 1.0, 0.0],
    ], dtype=np.float64)


# ------------------------------------------------------------------- the map

def load_marker_map(path):
    """Parse and validate marker_map.yaml. Raises ValueError on anything odd.

    Loud failure is the point: a typo in a board pose does not look like a bug
    later, it looks like the robot being confidently in the wrong room.
    """
    with open(str(path), 'r') as handle:
        document = yaml.safe_load(handle)
    if not isinstance(document, dict):
        raise ValueError('marker map is not a YAML mapping: %s' % path)
    for key in REQUIRED_TOP_LEVEL:
        if key not in document:
            raise ValueError('marker map is missing "%s": %s' % (key, path))

    boards = document['boards']
    if not isinstance(boards, dict) or not boards:
        raise ValueError('marker map has no boards: %s' % path)

    claimed = {}
    for name, board in boards.items():
        _validate_board(name, board)
        for marker_id in board['ids'].keys():
            if marker_id in claimed:
                raise ValueError(
                    'marker id %d is claimed by both "%s" and "%s"; an id may '
                    'only appear once or a detection is ambiguous'
                    % (marker_id, claimed[marker_id], name))
            claimed[marker_id] = name
    return document


def _validate_board(name, board):
    if not isinstance(board, dict):
        raise ValueError('board "%s" is not a mapping' % name)
    for key in ('tile_size', 'tile_spacing', 'ids', 'pose'):
        if key not in board:
            raise ValueError('board "%s" is missing "%s"' % (name, key))

    size = float(board['tile_size'])
    spacing = float(board['tile_spacing'])
    if size <= 0.0:
        raise ValueError('board "%s": tile_size must be positive' % name)
    if spacing < size:
        raise ValueError(
            'board "%s": tile_spacing (%.4f) is centre-to-centre and cannot be '
            'smaller than tile_size (%.4f) -- the tiles would overlap. Did you '
            'measure the gap between them instead? spacing = size + gap.'
            % (name, spacing, size))

    ids = board['ids']
    if not isinstance(ids, dict) or not ids:
        raise ValueError('board "%s": ids must be a non-empty mapping' % name)
    for marker_id, slot in ids.items():
        if not isinstance(marker_id, int):
            raise ValueError(
                'board "%s": marker id %r must be an integer' % (name, marker_id))
        if slot not in _SLOT_OFFSETS:
            raise ValueError(
                'board "%s": id %d has unknown slot "%s"; expected one of %s'
                % (name, marker_id, slot, sorted(_SLOT_OFFSETS)))

    pose = board['pose']
    if not isinstance(pose, dict):
        raise ValueError('board "%s": pose must be a mapping' % name)
    for key in ('x', 'y', 'yaw_deg'):
        if key not in pose:
            raise ValueError('board "%s": pose is missing "%s"' % (name, key))
    for key in ('x', 'y', 'z', 'yaw_deg'):
        if key in pose and not math.isfinite(float(pose[key])):
            raise ValueError('board "%s": pose.%s is not finite' % (name, key))


def board_for_ids(marker_map, detected_ids):
    """Pick the board with the most detected tiles. Returns (name, board, ids).

    Detections whose id is not in the map are dropped -- an unknown marker is
    not evidence about where the robot is, and this is also the cheapest
    rejection of a false positive.
    """
    best_name = None
    best_count = 0
    for name, board in marker_map['boards'].items():
        count = sum(1 for i in detected_ids if i in board['ids'])
        if count > best_count:
            best_name, best_count = name, count
    if best_name is None:
        return None, None, []
    board = marker_map['boards'][best_name]
    return best_name, board, [i for i in detected_ids if i in board['ids']]


def map_from_board(board):
    """T_map_board (4x4) for a board entry from the YAML."""
    pose = board['pose']
    yaw = math.radians(float(pose['yaw_deg']))
    translation = (float(pose['x']), float(pose['y']), float(pose.get('z', 0.0)))
    return make_transform(wall_board_rotation(yaw), translation)


# ------------------------------------------------------------------ geometry

def tile_object_points(board, marker_id):
    """The 4 corners of one tile in the BOARD frame, in OpenCV corner order."""
    slot = board['ids'][marker_id]
    offset_x, offset_y = _SLOT_OFFSETS[slot]
    spacing = float(board['tile_spacing'])
    half = float(board['tile_size']) / 2.0
    centre_x, centre_y = offset_x * spacing, offset_y * spacing
    return [(centre_x + sx * half, centre_y + sy * half, 0.0)
            for sx, sy in _TILE_CORNER_SIGNS]


def assemble_correspondences(board, detected_ids, detected_corners):
    """Stack every detected tile's corners into one PnP problem.

    Solving the four tiles together rather than one at a time is the single
    biggest accuracy win available here. A lone 60 mm square subtends a ~25 px
    baseline at 1 m, and its pose has a near-degenerate second solution that
    flips it about the vertical -- the classic planar-PnP ambiguity, which
    shows up as heading that snaps between two values tens of degrees apart.
    Four tiles spread over ~140 mm quadruple the baseline and break that
    symmetry outright.
    """
    object_points = []
    image_points = []
    used = []
    for marker_id, corners in zip(detected_ids, detected_corners):
        if marker_id not in board['ids']:
            continue
        flat = np.asarray(corners, dtype=np.float64).reshape(-1)
        if flat.size != 8 or not np.isfinite(flat).all():
            continue
        object_points.extend(tile_object_points(board, marker_id))
        image_points.extend([(flat[i], flat[i + 1]) for i in range(0, 8, 2)])
        used.append(marker_id)
    if not used:
        return None, None, []
    return (np.array(object_points, dtype=np.float64),
            np.array(image_points, dtype=np.float64),
            used)


def scale_intrinsics(camera_matrix, calibrated_size, actual_size):
    """Rescale fx, fy, cx, cy when the stream is not the calibrated size.

    Distortion coefficients act on normalised coordinates and do not scale.
    """
    cal_w, cal_h = calibrated_size
    act_w, act_h = actual_size
    if cal_w <= 0 or cal_h <= 0:
        raise ValueError('calibration size must be positive')
    matrix = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3).copy()
    if (cal_w, cal_h) == (act_w, act_h):
        return matrix
    sx, sy = float(act_w) / float(cal_w), float(act_h) / float(cal_h)
    matrix[0, 0] *= sx
    matrix[0, 2] *= sx
    matrix[1, 1] *= sy
    matrix[1, 2] *= sy
    return matrix


def solve_board_pose(object_points, image_points, camera_matrix, dist_coeffs):
    """PnP for the board in the camera optical frame.

    Returns (T_camopt_board, reprojection_error_px) or (None, None).

    IPPE_SQUARE is what the docking detector uses, but it is only defined for
    a single planar square of exactly four points. With several tiles stacked
    into one problem the general iterative solver is the correct choice, and
    it is also the one that benefits from the extra points.
    """
    if object_points is None or len(object_points) < 4:
        return None, None

    flags = (cv2.SOLVEPNP_IPPE_SQUARE if len(object_points) == 4
             else cv2.SOLVEPNP_ITERATIVE)

    try:
        ok, rvec, tvec = cv2.solvePnP(
            object_points.reshape(-1, 1, 3), image_points.reshape(-1, 1, 2),
            camera_matrix, dist_coeffs, flags=flags)
    except cv2.error:
        return None, None
    if not ok or not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
        return None, None
    # Behind the camera is not a pose, it is a failed solve.
    if float(tvec[2]) <= 0.0:
        return None, None

    projected, _ = cv2.projectPoints(
        object_points.reshape(-1, 1, 3), rvec, tvec, camera_matrix, dist_coeffs)
    residual = projected.reshape(-1, 2) - image_points.reshape(-1, 2)
    error = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))
    if not math.isfinite(error):
        return None, None

    rotation, _ = cv2.Rodrigues(rvec)
    return make_transform(rotation, tvec.reshape(3)), error


def robot_pose_in_map(map_from_board_tf, base_from_board_tf):
    """T_map_base -> (x, y, yaw). The line this whole module exists for."""
    map_from_base = map_from_board_tf.dot(invert_transform(base_from_board_tf))
    return (float(map_from_base[0, 3]),
            float(map_from_base[1, 3]),
            rotation_to_yaw(map_from_base[:3, :3]))


def corners_are_central(image_points, width, height, margin_fraction):
    """True if every corner sits inside the central box of the image.

    This lens is ~115 degrees horizontal, and a 5-coefficient plumb-bob model
    fits its edges poorly. A board detected in the corner of the frame still
    reprojects cleanly -- the distortion error is absorbed into a slightly
    wrong pose, which is exactly the failure that passes every other check.
    """
    if margin_fraction <= 0.0:
        return True
    points = np.asarray(image_points, dtype=np.float64).reshape(-1, 2)
    low_x, high_x = width * margin_fraction, width * (1.0 - margin_fraction)
    low_y, high_y = height * margin_fraction, height * (1.0 - margin_fraction)
    return bool(np.all((points[:, 0] >= low_x) & (points[:, 0] <= high_x)
                       & (points[:, 1] >= low_y) & (points[:, 1] <= high_y)))


def poses_agree(poses, position_tolerance, yaw_tolerance):
    """True if every (x, y, yaw) in `poses` is within tolerance of their mean.

    One bad fix that reseeds AMCL is worse than no fix at all, because the
    filter then spends its recovery budget converging onto a lie. Requiring a
    few consecutive frames to agree is the cheapest defence there is.
    """
    if len(poses) < 2:
        return False
    array = np.asarray(poses, dtype=np.float64)
    mean_x, mean_y = float(np.mean(array[:, 0])), float(np.mean(array[:, 1]))
    # Circular mean: yaw near +/-pi averages to zero the naive way.
    mean_yaw = math.atan2(float(np.mean(np.sin(array[:, 2]))),
                          float(np.mean(np.cos(array[:, 2]))))
    for x, y, yaw in array:
        if math.hypot(x - mean_x, y - mean_y) > position_tolerance:
            return False
        if abs(math.atan2(math.sin(yaw - mean_yaw),
                          math.cos(yaw - mean_yaw))) > yaw_tolerance:
            return False
    return True


def mean_pose(poses):
    """Mean of (x, y, yaw) triples, with the yaw averaged circularly."""
    array = np.asarray(poses, dtype=np.float64)
    return (float(np.mean(array[:, 0])),
            float(np.mean(array[:, 1])),
            math.atan2(float(np.mean(np.sin(array[:, 2]))),
                       float(np.mean(np.cos(array[:, 2])))))
