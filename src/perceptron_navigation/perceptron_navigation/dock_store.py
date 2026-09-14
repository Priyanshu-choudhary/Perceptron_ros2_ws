"""Persist a dock pose found during exploration, and the staging pose for it.

WHAT IS SAVED
The marker pose in the map frame, plus the map the run built. The map matters:
a pose in `map` only means anything if `map` means the same thing when you read
it back. Under SLAM the map frame is anchored to wherever the robot started, so
a dock pose from one run is meaningless in the next unless that run relocalises
against the same map. Saving the map path with the pose is what makes reuse
across runs possible at all; within one run it is simply unused.
"""

import json
import math
from pathlib import Path

DEFAULT_PATH = '~/.ros/perceptron_dock/dock.json'


def marker_normal(quaternion):
    """Map-frame unit vector out of the marker face, flattened into the plane.

    solvePnP puts the marker's model points in its own XY plane, so +Z is the
    face normal. The robot drives on a floor, so only the horizontal part is
    useful; a marker mounted with any tilt still gives a sane approach line.
    """
    x, y, z, w = quaternion
    nx = 2.0 * (x * z + w * y)
    ny = 2.0 * (y * z - w * x)
    norm = math.hypot(nx, ny)
    if norm < 1e-6:
        raise ValueError('Marker normal is vertical; cannot derive an approach')
    return nx / norm, ny / norm


def staging_pose(position, quaternion, standoff):
    """Where to park before handing over to the docking controller.

    `standoff` metres out along the marker normal, facing back at the marker.
    This is deliberately further out than the docking controller's own pre-dock
    pose: Nav2 only has to get close, and the docking controller then does its
    own detection and precision approach from there.
    """
    nx, ny = marker_normal(quaternion)
    return (position[0] + standoff * nx,
            position[1] + standoff * ny,
            math.atan2(-ny, -nx))


def save(path, record):
    destination = Path(path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix('.tmp')
    temporary.write_text(json.dumps(record, indent=2, allow_nan=False) + '\n',
                         encoding='utf-8')
    # Replace atomically: a half-written dock pose is worse than none, because
    # the robot would drive somewhere confidently wrong.
    temporary.replace(destination)
    return str(destination)


def load(path):
    try:
        record = json.loads(Path(path).expanduser().read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None
    position = record.get('position')
    orientation = record.get('orientation')
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        return None
    try:
        values = [float(position[k]) for k in ('x', 'y', 'z')]
        values += [float(orientation[k]) for k in ('x', 'y', 'z', 'w')]
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(v) for v in values):
        return None
    return record
