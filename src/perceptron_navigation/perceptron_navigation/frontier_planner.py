"""Reachable frontier and camera-viewpoint selection, independent of ROS.

Goals are always in observed free space. Unknown cells remain blocked for the
robot footprint: approaching their boundary grows SLAM without planning through
unseen walls. Four-connected reachability cannot cut diagonally through corners.
"""

from collections import deque
from dataclasses import dataclass
import math

import numpy as np
from scipy import ndimage


@dataclass
class Grid:
    data: np.ndarray
    resolution: float
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0

    def cell(self, x, y):
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (math.floor((-s * dx + c * dy) / self.resolution),
                math.floor((c * dx + s * dy) / self.resolution))

    def world(self, row, col):
        x, y = (col + 0.5) * self.resolution, (row + 0.5) * self.resolution
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return (self.origin_x + c * x - s * y,
                self.origin_y + s * x + c * y)


@dataclass
class Target:
    x: float
    y: float
    yaw: float
    kind: str
    gain: float = 0.0


@dataclass
class Plan:
    target: object
    frontier_cells: int
    reachable_cells: int
    reason: str


def free_line(free, start, end):
    """Conservative supercover visibility through free cells only."""
    r0, c0 = start
    r1, c1 = end
    steps = max(abs(r1 - r0), abs(c1 - c0)) * 2 + 1
    rows = np.linspace(r0, r1, steps)
    cols = np.linspace(c0, c1, steps)
    for rr, cc in zip(rows, cols):
        for r in (math.floor(rr), math.ceil(rr)):
            for c in (math.floor(cc), math.ceil(cc)):
                if not free[r, c]:
                    return False
    return True


def reachable_distances(safe, start):
    distances = np.full(safe.shape, -1, dtype=np.int32)
    r, c = start
    if not (0 <= r < safe.shape[0] and 0 <= c < safe.shape[1] and safe[r, c]):
        return distances
    distances[r, c] = 0
    queue = deque([(r, c)])
    while queue:
        r, c = queue.popleft()
        for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
            if (0 <= nr < safe.shape[0] and 0 <= nc < safe.shape[1]
                    and safe[nr, nc] and distances[nr, nc] < 0):
                distances[nr, nc] = distances[r, c] + 1
                queue.append((nr, nc))
    return distances


def nearest_safe(safe, start, max_cells):
    """Closest safe cell to `start`, searched outward, or None.

    The robot's own cell often fails the footprint test for reasons that have
    nothing to do with being stuck: unknown cells count as blocking, so standing
    anywhere near an unmapped pocket - which is most of the time, early in a run
    - makes the cell "unsafe" while the robot sits there perfectly happily.
    Seeding the flood fill from the nearest genuinely safe cell keeps the run
    alive instead of declaring the robot lost.
    """
    r0, c0 = start
    rows, cols = safe.shape
    if 0 <= r0 < rows and 0 <= c0 < cols and safe[r0, c0]:
        return (r0, c0)
    for radius in range(1, int(max_cells) + 1):
        best = None
        best_d = None
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if max(abs(dr), abs(dc)) != radius:
                    continue
                r, c = r0 + dr, c0 + dc
                if not (0 <= r < rows and 0 <= c < cols) or not safe[r, c]:
                    continue
                d = dr * dr + dc * dc
                if best_d is None or d < best_d:
                    best, best_d = (r, c), d
        if best is not None:
            return best
    return None


def choose_target(grid, robot_xy, excluded=(), scanned=(), search=False,
                  robot_radius=0.33, clearance_margin=0.05,
                  frontier_standoff=0.65, min_frontier_cells=5,
                  min_goal_distance=0.25, viewpoint_spacing=1.2,
                  frontier_gain_weight=2.5, seed_search_radius=1.0,
                  free_threshold=25):
    """Return a frontier goal, then uninspected room viewpoints for search.

    excluded: (map x, map y, radius) tuples for recently visited/failed goals.
    scanned: map positions where a full camera sweep has succeeded. Viewpoints
    across walls are not considered covered just because they are nearby.
    """
    if grid.resolution <= 0 or grid.data.ndim != 2 or not grid.data.size:
        raise ValueError('Invalid occupancy grid')
    free = (grid.data >= 0) & (grid.data <= free_threshold)
    unknown = grid.data < 0
    # Treat outside the current map as unknown, including when the publisher
    # crops its occupancy grid tightly around observed cells.
    padded_unknown = np.pad(unknown, 1, constant_values=True)
    adjacent_unknown = ndimage.binary_dilation(padded_unknown)[1:-1, 1:-1]
    frontier = free & adjacent_unknown
    clearance = ndimage.distance_transform_edt(np.pad(free, 1))[1:-1, 1:-1]
    # Cell-centre distances overestimate distance to a blocked cell's edge.
    safe = free & (clearance * grid.resolution >=
                   robot_radius + clearance_margin + grid.resolution * 0.5)
    seed = nearest_safe(safe, grid.cell(*robot_xy),
                        max(1, round(seed_search_radius / grid.resolution)))
    frontier_count = int(frontier.sum())
    if seed is None:
        return Plan(None, frontier_count, 0, 'robot_has_no_clearance')
    distance = reachable_distances(safe, seed)
    reachable = distance >= 0
    reachable_count = int(reachable.sum())
    if not reachable_count:
        return Plan(None, frontier_count, 0, 'robot_has_no_clearance')

    rows, cols = np.indices(free.shape)
    local_x, local_y = (cols + 0.5) * grid.resolution, (rows + 0.5) * grid.resolution
    c, s = math.cos(grid.origin_yaw), math.sin(grid.origin_yaw)
    wx = grid.origin_x + c * local_x - s * local_y
    wy = grid.origin_y + s * local_x + c * local_y
    eligible = reachable & (np.hypot(wx - robot_xy[0], wy - robot_xy[1]) >= min_goal_distance)
    for x, y, radius in excluded:
        eligible &= np.hypot(wx - x, wy - y) > radius

    labels, count = ndimage.label(frontier, structure=np.ones((3, 3)))
    best, best_score = None, float('inf')
    for label in range(1, count + 1):
        cluster = labels == label
        size = int(cluster.sum())
        if size < min_frontier_cells:
            continue
        separation, nearest = ndimage.distance_transform_edt(~cluster, return_indices=True)
        candidates = eligible & (separation * grid.resolution <= frontier_standoff)
        rr, cc = np.nonzero(candidates)
        if not len(rr):
            continue
        # Travel cost minus information gain, both in metres.
        #
        # The gain term used to be capped at 3.0 m and weighted 0.25, so it could
        # never shift the choice by more than 0.75 m. Distance therefore always
        # won and the robot nibbled at whatever frontier happened to be nearest,
        # advancing about 0.3 m per goal and running a four-spin camera sweep at
        # each one. Measured in room_world it managed five goals inside a 1.2 m
        # radius of its spawn before the run died. A real frontier a few metres
        # away has to be able to beat a one-cell scrap underfoot.
        scores = (distance[rr, cc] * grid.resolution
                  - frontier_gain_weight * size * grid.resolution)
        for i in np.argsort(scores):
            r, col = int(rr[i]), int(cc[i])
            fr, fc = int(nearest[0, r, col]), int(nearest[1, r, col])
            if not free_line(free, (r, col), (fr, fc)):
                continue
            if scores[i] < best_score:
                x, y = grid.world(r, col)
                fx, fy = grid.world(fr, fc)
                best = Target(x, y, math.atan2(fy - y, fx - x), 'frontier',
                              size * grid.resolution)
                best_score = scores[i]
            break
    if best is not None:
        return Plan(best, frontier_count, reachable_count, 'frontier')

    if search:
        # Sample reachable room interiors even when the laser has already
        # mapped them from a distant doorway but the camera has not inspected.
        stride = max(1, int(viewpoint_spacing / (2.0 * grid.resolution)))
        rr, cc = np.nonzero(eligible)
        order = np.argsort(distance[rr, cc])
        for i in order:
            r, col = int(rr[i]), int(cc[i])
            if r % stride or col % stride:
                continue
            x, y = grid.world(r, col)
            covered = False
            for sx, sy in scanned:
                sr, sc = grid.cell(sx, sy)
                if (math.hypot(x - sx, y - sy) < viewpoint_spacing
                        and 0 <= sr < free.shape[0] and 0 <= sc < free.shape[1]
                        and free_line(free, (r, col), (sr, sc))):
                    covered = True
                    break
            if not covered:
                return Plan(Target(x, y, 0.0, 'viewpoint'), frontier_count,
                            reachable_count, 'viewpoint')
    reason = 'no_frontiers' if frontier_count == 0 else 'no_reachable_untried_targets'
    return Plan(None, frontier_count, reachable_count, reason)
