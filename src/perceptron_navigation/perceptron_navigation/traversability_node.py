#!/usr/bin/env python3
"""Turn a 3D point cloud into a traversability grid.

Subscribes  /points                     sensor_msgs/PointCloud2 (3D lidar or RGBD)
Publishes   /traversability_grid        nav_msgs/OccupancyGrid, for RViz and debugging
            /traversability_obstacles   sensor_msgs/PointCloud2, for the Nav2 costmap

WHY THIS EXISTS
A 2D scan cannot represent the two hazards this arena actually contains:

  Rocks    are positive obstacles. A planar scan sees them only if they happen
           to intersect its one height, so a boulder lower than the scan plane
           is invisible and one taller than it looks like a wall of unknown
           extent.
  Craters  are NEGATIVE obstacles and a planar scan can never see them at all.
           The beam passes over the hole and returns whatever lies beyond, so
           the costmap reads "free" over a pit the robot will fall into. In this
           competition that is a 30-point penalty per crossing.

So the cloud is binned into a 2.5D elevation grid and each cell is classified by
what it would do to the robot, not by whether something reflected a beam.

HOW A CELL BECOMES LETHAL
  step       max z - min z inside the cell. A rock face, or the lip of a crater.
  slope      gradient of mean z against neighbouring cells. Sand the robot can
             climb versus sand it will slide down.
  negative   mean z sitting below the surrounding ground plane. This is the
             crater test, and it is the one a planar scan cannot do.
  unknown    no returns at all. Deliberately NOT treated as free: the inside of
             a crater and the far side of a boulder both produce no returns, and
             calling that free is exactly the failure this node exists to stop.

WHY IT PUBLISHES A POINT CLOUD AS WELL
Nav2's stock ObstacleLayer takes PointCloud2 observation sources, so emitting
one lethal point per bad cell drops straight into the existing costmap with no
custom C++ layer to build and maintain. The OccupancyGrid is the human-readable
version of the same decision.
"""

import math
import struct

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Decode a PointCloud2 into an (N, 3) float array, dropping non-finite points."""
    offsets = {f.name: (f.offset, f.datatype) for f in msg.fields}
    for axis in ('x', 'y', 'z'):
        if axis not in offsets:
            return np.empty((0, 3), dtype=np.float32)
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    n = msg.width * msg.height
    if n == 0 or raw.size < n * msg.point_step:
        return np.empty((0, 3), dtype=np.float32)
    raw = raw[:n * msg.point_step].reshape(n, msg.point_step)
    out = np.empty((n, 3), dtype=np.float32)
    for i, axis in enumerate(('x', 'y', 'z')):
        off = offsets[axis][0]
        out[:, i] = raw[:, off:off + 4].copy().view(np.float32).ravel()
    return out[np.isfinite(out).all(axis=1)]


class TraversabilityNode(Node):

    def __init__(self):
        super().__init__('traversability_node')

        self.declare_parameter('input_topic', '/points')
        self.declare_parameter('grid_topic', '/traversability_grid')
        self.declare_parameter('obstacle_topic', '/traversability_obstacles')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 5.0)

        # Grid geometry. A local, robot-centred window: this is a perception
        # product, not a map, and Nav2's costmap does the accumulating.
        self.declare_parameter('grid_size', 8.0)          # metres, square
        self.declare_parameter('resolution', 0.10)        # metres per cell

        # Classification thresholds, all in metres or radians.
        self.declare_parameter('max_step', 0.06)          # rock face the wheels cannot climb
        self.declare_parameter('max_slope', 0.35)         # rad, about 20 degrees
        self.declare_parameter('crater_depth', 0.05)      # below local ground = hole
        self.declare_parameter('robot_clearance', 0.031)  # chassis underside above sand
        self.declare_parameter('min_points_per_cell', 2)
        self.declare_parameter('unknown_is_lethal', False)
        # Beyond this range the beam angle is so shallow that a crater cannot be
        # resolved, so cells further out are reported unknown rather than free.
        self.declare_parameter('max_range', 5.0)

        g = self.get_parameter
        self.robot_frame = g('robot_frame').value
        self.size = float(g('grid_size').value)
        self.res = float(g('resolution').value)
        self.max_step = float(g('max_step').value)
        self.max_slope = float(g('max_slope').value)
        self.crater_depth = float(g('crater_depth').value)
        self.min_pts = int(g('min_points_per_cell').value)
        self.unknown_lethal = bool(g('unknown_is_lethal').value)
        self.max_range = float(g('max_range').value)

        self.n = int(round(self.size / self.res))
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.grid_pub = self.create_publisher(OccupancyGrid, g('grid_topic').value, 1)
        self.obs_pub = self.create_publisher(PointCloud2, g('obstacle_topic').value, 1)
        self.create_subscription(PointCloud2, g('input_topic').value,
                                 self._cloud_cb, qos_profile_sensor_data)

        self._latest = None
        self._seen = 0
        self.create_timer(1.0 / max(0.5, float(g('publish_rate').value)), self._process)
        self.create_timer(20.0, self._warn_if_silent)

        self.get_logger().info(
            f'Traversability grid up: {self.n}x{self.n} cells at {self.res:.2f} m, '
            f'step>{self.max_step:.3f} m or slope>{math.degrees(self.max_slope):.0f} deg '
            f'or {self.crater_depth:.3f} m below grade is lethal.')

    def _cloud_cb(self, msg: PointCloud2):
        self._latest = msg
        self._seen += 1

    def _warn_if_silent(self):
        if self._seen == 0:
            self.get_logger().warn(
                f'Nothing on {self.get_parameter("input_topic").value} yet. The 3D '
                'lidar is only fitted when the robot is launched with '
                'lidar_type:=3d or both.', throttle_duration_sec=30.0)

    def _process(self):
        msg = self._latest
        if msg is None:
            return

        pts = cloud_to_xyz(msg)
        if pts.shape[0] == 0:
            return

        # Into the gravity-aligned robot frame. Doing this in the sensor frame
        # would mix the lidar's downward tilt into every slope estimate.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.robot_frame, msg.header.frame_id, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f'No transform to {self.robot_frame}: {exc}',
                                   throttle_duration_sec=5.0)
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        # quaternion -> rotation matrix
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float32)
        pts = pts @ R.T + np.array([t.x, t.y, t.z], dtype=np.float32)

        half = self.size / 2.0
        keep = ((np.abs(pts[:, 0]) < half) & (np.abs(pts[:, 1]) < half)
                & (np.hypot(pts[:, 0], pts[:, 1]) < self.max_range))
        pts = pts[keep]
        if pts.shape[0] == 0:
            return

        ix = ((pts[:, 0] + half) / self.res).astype(np.int32)
        iy = ((pts[:, 1] + half) / self.res).astype(np.int32)
        np.clip(ix, 0, self.n - 1, out=ix)
        np.clip(iy, 0, self.n - 1, out=iy)
        flat = iy * self.n + ix
        cells = self.n * self.n

        count = np.bincount(flat, minlength=cells)
        zsum = np.bincount(flat, weights=pts[:, 2], minlength=cells)
        zmax = np.full(cells, -np.inf, dtype=np.float32)
        zmin = np.full(cells, np.inf, dtype=np.float32)
        np.maximum.at(zmax, flat, pts[:, 2])
        np.minimum.at(zmin, flat, pts[:, 2])

        known = count >= self.min_pts
        mean = np.where(known, zsum / np.maximum(count, 1), np.nan)
        step = np.where(known, zmax - zmin, np.nan)

        grid_mean = mean.reshape(self.n, self.n)
        grid_step = step.reshape(self.n, self.n)

        # Local ground level: the median of the known cells. On a 9 x 5 m sand
        # arena the ground really is flat, so a single reference is honest here;
        # on a slope this would need a fitted plane instead.
        finite = grid_mean[np.isfinite(grid_mean)]
        ground = float(np.median(finite)) if finite.size else 0.0

        # Slope from the gradient of the mean-height surface.
        filled = np.where(np.isfinite(grid_mean), grid_mean, ground)
        gy, gx = np.gradient(filled, self.res)
        slope = np.arctan(np.hypot(gx, gy))

        lethal = np.zeros((self.n, self.n), dtype=bool)
        lethal |= np.isfinite(grid_step) & (grid_step > self.max_step)      # rocks
        lethal |= slope > self.max_slope                                    # steep sand
        lethal |= (np.isfinite(grid_mean)
                   & (grid_mean < ground - self.crater_depth))              # craters

        unknown = ~np.isfinite(grid_mean)
        if self.unknown_lethal:
            lethal |= unknown

        out = np.full((self.n, self.n), -1, dtype=np.int8)
        out[np.isfinite(grid_mean)] = 0
        out[lethal] = 100

        stamp = self.get_clock().now().to_msg()
        grid = OccupancyGrid()
        grid.header = Header(stamp=stamp, frame_id=self.robot_frame)
        grid.info.resolution = self.res
        grid.info.width = self.n
        grid.info.height = self.n
        grid.info.origin.position.x = -half
        grid.info.origin.position.y = -half
        grid.info.origin.orientation.w = 1.0
        grid.data = out.ravel().tolist()
        self.grid_pub.publish(grid)

        self._publish_obstacles(stamp, lethal, grid_mean, ground)

    def _publish_obstacles(self, stamp, lethal, grid_mean, ground):
        """One point per lethal cell, for Nav2's ObstacleLayer to mark.

        Each point is lifted to just above the sand rather than placed at the
        measured height. A crater floor is BELOW the ground plane, and a marking
        point down there would sit under the costmap's z window and be ignored,
        so the hole would silently stay traversable.
        """
        ys, xs = np.nonzero(lethal)
        if xs.size == 0:
            self.obs_pub.publish(self._empty_cloud(stamp))
            return
        half = self.size / 2.0
        px = (xs.astype(np.float32) + 0.5) * self.res - half
        py = (ys.astype(np.float32) + 0.5) * self.res - half
        pz = np.full(xs.size, ground + 0.10, dtype=np.float32)

        data = bytearray()
        for i in range(xs.size):
            data += struct.pack('<fff', float(px[i]), float(py[i]), float(pz[i]))

        cloud = PointCloud2()
        cloud.header = Header(stamp=stamp, frame_id=self.robot_frame)
        cloud.height = 1
        cloud.width = int(xs.size)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * cloud.width
        cloud.is_dense = True
        cloud.data = bytes(data)
        self.obs_pub.publish(cloud)

    def _empty_cloud(self, stamp):
        cloud = PointCloud2()
        cloud.header = Header(stamp=stamp, frame_id=self.robot_frame)
        cloud.height = 1
        cloud.width = 0
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = 0
        cloud.is_dense = True
        cloud.data = b''
        return cloud


def main(args=None):
    rclpy.init(args=args)
    node = TraversabilityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
