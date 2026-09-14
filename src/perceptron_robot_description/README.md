# perceptron_robot_description

The robot's geometry: URDF/xacro, STL meshes, frames, and the Gazebo sensor and
friction definitions. Every other package derives its numbers from here, so a
mistake in this package shows up as a bug somewhere else entirely.

```
urdf/
  perceptron_robot.xacro     link/joint tree, ros2_control block   <- start here
  perceptron_robot.gazebo    sensors, friction, materials (included by the xacro)
  materials.xacro            named RViz colours
  perceptron_robot.trans     Fusion export leftover, unused
meshes/
  base_link.stl  rim_1.stl  tyre.stl  camera.stl  IMU.stl
  *_1.stl *_5.stl *_6.stl *_7.stl     duplicate Fusion exports, unused
launch/
  display.launch.py          RViz + joint sliders, no physics
```

---

## Frames

```
base_footprint                      on the ground, centre of the 4 wheels
   └─ base_link                     +0.0625 m in z (wheel axle height)
        ├─ camera_link              +X out of the lens, +Z up
        │    └─ camera_optical_link +Z out of the lens, +X right, +Y down
        ├─ laser_link               LiDAR scan plane, +X forward
        ├─ lidar_post_link          the post it stands on
        ├─ imu_link
        ├─ left_front_wheel         continuous, axis 0 1 0
        ├─ right_front_wheel
        ├─ left_back_wheel
        └─ right_back_wheel
```

Two conventions that trip people up, both of which caused real bugs here:

**`base_footprint` must be under the robot.** It is the frame the whole stack
reasons in: `diff_drive_controller` reports odometry for it, and the docking
controller expresses the marker in it and treats `x` as forward and `y` as left.
Originally `base_link` sat at a *corner* of the Fusion chassis mesh, about
0.11 m behind and 0.15 m to the side of the real robot centre, which silently
biased every docking measurement. It is now at the centre of the four wheels.

**`camera_link` and `camera_optical_link` are two different things.** Gazebo's
camera sensor looks down the link's **+X**, with +Z up. OpenCV's `solvePnP`
returns poses in an *optical* frame: **+Z out of the lens, +X right, +Y down**.
The fixed joint between them is the standard `rpy="-pi/2 0 -pi/2"` conversion.
Mixing them up means a pose whose `x` you think is "forward" is really "right",
and the robot drives sideways. `camera_optical_link` is what the Gazebo camera
plugin stamps its images with and what the ArUco node works in.

Verify the conversion any time you touch it:

```bash
ros2 run tf2_ros tf2_echo camera_link camera_optical_link
```

---

## Measured geometry

Everything below was measured from the STL bounding boxes, not guessed. These
are the numbers `perceptron_robot_control` and `perceptron_docking` are tuned
against.

| quantity | value | where it comes from |
| --- | --- | --- |
| Wheel radius | 0.0625 m | `tyre.stl` is a 125 mm diameter cylinder |
| Wheel width | 0.0579 m | `tyre.stl` |
| Track (left to right) | 0.37762 m | 2 × `wheel_y` |
| Wheelbase (front to back) | 0.21548 m | 2 × `wheel_x` |
| Chassis box | 0.415 × 0.4526 × 0.111 m | `base_link.stl` bounding box |
| Ground clearance | 0.0311 m | chassis underside above the ground |
| `base_footprint` to front bumper | 0.1961 m | chassis box front face |
| `base_footprint` to camera | 0.18314 m, at 0.1621 m height | `camera.stl` centre |
| LiDAR scan plane | 0.21 m above ground, on the centre of rotation | `lidar_z` in the xacro |
| Robot mass | 5.2832 kg chassis + 4 × 0.71244 kg wheels | Fusion inertial data |

### Re-deriving them if the CAD changes

The STLs are exported in the Fusion **assembly** frame, not per-link, which is
why every `<visual>` carries a large constant offset like
`xyz="-0.175 -0.7 0"`. Those offsets are not noise and must not be "cleaned up":
they are what moves the mesh from assembly coordinates onto its link.

To recompute them after a new export, read each mesh's bounding box and work out
the transform that lands its centre on the link origin:

```bash
python3 - <<'PY'
import struct, numpy as np
def bbox(path):
    d = open(path, 'rb').read()
    n = struct.unpack('<I', d[80:84])[0]
    v = np.frombuffer(d[84:], dtype=np.uint8).reshape(n, 50)[:, 12:48]
    v = v.copy().view('<f4').reshape(-1, 3)
    return v.min(0), v.max(0), (v.min(0) + v.max(0)) / 2
for f in ('base_link', 'rim_1', 'tyre', 'camera', 'IMU'):
    lo, hi, c = bbox(f'meshes/{f}.stl')
    print(f'{f:10s} min={np.round(lo,3)} max={np.round(hi,3)} '
          f'size={np.round(hi-lo,3)} centre={np.round(c,3)}  (mm)')
PY
```

For the wheels the visual origin is `rpy="0 0 pi/2"` (which maps the mesh's +X
axis, the wheel's spin axis, onto the link's +Y) with
`xyz = -Rz(pi/2) · mesh_centre`. That is where `-0.175 -0.7 0` comes from.

---

## Xacro arguments

```bash
xacro perceptron_robot.xacro is_sim:=true simple_visuals:=false
```

| argument | default | effect |
| --- | --- | --- |
| `is_sim` | `true` | `true` includes `perceptron_robot.gazebo` and the `gazebo_ros2_control/GazeboSystem` hardware plugin. `false` swaps in `mock_components/GenericSystem`, for RViz-only and hardware bringup. |
| `simple_visuals` | `false` | `true` replaces the ~340k-triangle STL visuals with boxes and cylinders. Collisions and inertias are unchanged, so **physics is identical** — this is purely a rendering cost switch, and a large one on WSL. |
| `gazebo_controllers` | `perceptron_robot_control/config/controllers.yaml` | Path handed to the `gazebo_ros2_control` plugin. |

---

## Collisions: primitives, not meshes

Visuals use the STL meshes. Collisions are a box for the chassis and cylinders
for the wheels. This is deliberate:

- A 34k-triangle mesh collision on a body that spins at 3 rad/s is expensive and
  numerically fragile in ODE.
- The original export placed each `tyre_*` link 0.22 to 0.42 m away from the
  `rim_*` link it was attached to. Gazebo lumps fixed joints into their parent,
  so each wheel became a torus collision swinging on a 0.22 m arm, ploughing
  through the ground and throwing the robot across the world.

Each wheel is now a single link with a cylinder collision concentric with its
joint, and the rim plus tyre meshes as visuals on top.

---

## The gazebo_ros2_control comment trap

**Do not put a colon followed by a space, or a line starting with a dash and a
space, anywhere in this URDF — including inside XML comments.**

`gazebo_ros2_control` on Humble hands the entire generated URDF to rcl as a
`robot_description` parameter override, which is parsed as a YAML scalar. Either
sequence terminates the scalar early and the parse fails with:

```
Couldn't parse parameter override rule: '--param robot_description:=<?xml ...
```

The controller manager is then never created, both spawners time out after 60 s,
and nothing in the error message points at a comment. There is a warning to this
effect at the top of `perceptron_robot.xacro`. Check after any edit:

```bash
xacro urdf/perceptron_robot.xacro is_sim:=true | grep -nE ": |:$"
```

Silence means you are fine.

---

## Sensors (`perceptron_robot.gazebo`)

Simulation-only; ignored when `is_sim:=false`.

| sensor | rate | topics | notes |
| --- | --- | --- | --- |
| Camera | 10 Hz | `/camera/image_raw`, `/camera/camera_info` | 800×600, 80° horizontal FOV, frame `camera_optical_link` |
| LiDAR | 10 Hz | `/scan` | LDROBOT D500: 500 points, 360°, 0.05–12 m, frame `laser_link` |
| IMU | 100 Hz | `/imu/data_raw` | frame `imu_link` |

The LiDAR sits at 0.21 m over the centre of rotation. Both parts of that matter:
on the centre of rotation so turning in place produces no lidar translation
(which keeps SLAM scan matching well conditioned), and at 0.21 m so it clears
the camera body at 0.1821 m instead of carving a blind wedge out of every scan.
Its Gazebo sensor is `type="ray"` and not `gpu_ray`, because CPU ray casting
works headless and under WSL software GL where `gpu_ray` silently returns
nothing. See [`perceptron_navigation`](../perceptron_navigation/README.md).

The camera resolution is a deliberate compromise between two limits, both
documented in the file: big enough that a 0.15 m marker at 2 m seen 40° off-axis
still spans ~28 px (ArUco needs roughly 4 px per module), and small enough that
the uncompressed stream does not swamp the middleware. Raw images are not
compressed — 1280×960 at 15 Hz is 55 MB/s, which starves the ROS graph on a 4 GB
WSL VM badly enough that new nodes fail to discover anything.

### Wheel friction

```xml
<mu1>1.0</mu1> <mu2>0.2</mu2> <fdir1>1 0 0</fdir1>
```

`fdir1` is the wheel's rolling direction (its local +X, because it spins about
+Y); `mu1` acts along it and `mu2` across it. A low `mu2` is what lets a 4-wheel
skid-steer rotate instead of juddering and hopping — all four wheels must scrub
sideways for the robot to turn at all.

---

## Checking a change

```bash
# 1. Does it parse, and is the tree sane?
xacro urdf/perceptron_robot.xacro is_sim:=true > /tmp/r.urdf && check_urdf /tmp/r.urdf

# 2. The comment trap
grep -nE ": |:$" /tmp/r.urdf

# 3. Does it look right?
ros2 launch perceptron_robot_description display.launch.py

# 4. Does it stand up in physics?
ros2 launch perceptron_robot_bringup gazebo_control2.launch.py
```

In RViz, turn on **TF** and **RobotModel** with *Collision Enabled* — the fastest
way to spot a collision shape that has wandered off from its visual.

`display.launch.py` processes the xacro with default arguments, i.e.
`is_sim:=true`, so the Gazebo tags are present but harmless. `use_gui:=true`
(the default) runs `joint_state_publisher_gui` for the wheel sliders; pass
`use_gui:=false` for the plain publisher.
