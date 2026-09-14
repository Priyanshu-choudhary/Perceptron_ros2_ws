# perceptron_robot_bringup

The launch files that wire the other packages together, plus the RViz layout.
No nodes of its own.

```
launch/
  gazebo_control2.launch.py   Gazebo + ros2_control + robot_state_publisher   <- the main one
  rviz_control.launch.py      mock hardware + controllers + RViz, no Gazebo
rviz/
  docking.rviz                RViz layout for the docking stack
```

---

## gazebo_control2.launch.py

Brings the robot up in Gazebo Classic with `ros2_control`. Everything else in
the simulation stacks on top of this.

```bash
ros2 launch perceptron_robot_bringup gazebo_control2.launch.py
```

### What it starts, and in what order

```
SetEnvironmentVariable   GAZEBO_MODEL_PATH, GAZEBO_RESOURCE_PATH,
                         GAZEBO_MODEL_DATABASE_URI=""
        |
        +-- robot_state_publisher     xacro -> /robot_description -> TF
        |
        +-- gzserver (world file)     loads libgazebo_ros2_control.so, which
        |                             creates the controller_manager INSIDE gzserver
        +-- gzclient                  (if gui:=true)
        |
        +-- [4 s] spawn_entity.py     pulls /robot_description into Gazebo
                   |
                   | on exit
                   v
              spawner joint_state_broadcaster
                   |
                   | on exit
                   v
              spawner diff_drive_controller
```

The ordering is not decoration. The controller manager does not exist until the
model — and therefore the `gazebo_ros2_control` plugin — has been spawned, so
the spawners are chained behind `spawn_entity` with `OnProcessExit` handlers
rather than started in parallel. The 4 s `TimerAction` before spawning gives
gzserver time to advertise `/spawn_entity`.

**There is deliberately no `ros2_control_node` here.** With `gazebo_ros2_control`
the controller manager is hosted by the Gazebo plugin; starting a second one
produces two managers fighting over the same joints. See
[`perceptron_robot_control/README.md`](../perceptron_robot_control/README.md).

### Arguments

| argument | default | effect |
| --- | --- | --- |
| `world_path` | `perceptron_robot_gazebo/worlds/docking_world.world` | World file to load. |
| `gui` | `true` | `false` skips `gzclient`. Headless is much faster and RViz still shows everything. |
| `use_sim_time` | `true` | Passed to every node started here. |
| `simple_visuals` | `false` | Forwarded to the xacro. `true` swaps the STL visuals for primitives; physics is unchanged. |
| `software_gl` | `false` | `true` sets `LIBGL_ALWAYS_SOFTWARE=1`. Roughly 10× slower — only if WSLg cannot give Gazebo a GL context (black window, or gzclient crashes). |
| `x`, `y`, `z`, `yaw` | `0, 0, 0.02, 0` | Spawn pose. `base_footprint` is on the ground, so a 2 cm drop is plenty; spawning higher makes the robot bounce. |

```bash
# fast headless, robot off to one side facing away from the dock
ros2 launch perceptron_robot_bringup gazebo_control2.launch.py \
     gui:=false simple_visuals:=true x:=0.4 y:=1.1 yaw:=-1.9
```

### The Gazebo environment variables

| variable | why |
| --- | --- |
| `GAZEBO_MODEL_PATH` | Must contain the dock models directory *and* the parent of the description package's share directory, so `package://perceptron_robot_description/meshes/...` resolves. |
| `GAZEBO_RESOURCE_PATH` | Same, for non-model resources. |
| `GAZEBO_MODEL_DATABASE_URI=""` | Empty on purpose. Stops Gazebo reaching for the online model database, which otherwise blocks startup for ~30 s. |

Meshes rendering as white boxes, or "Unable to find uri" in the gzserver log,
almost always means one of the first two is wrong.

---

## rviz_control.launch.py

Controllers against `mock_components/GenericSystem` (via `is_sim:=false` in the
xacro) with RViz, no Gazebo. The mock hardware echoes commands straight back as
state, so the robot moves in TF but there is no physics.

```bash
ros2 launch perceptron_robot_bringup rviz_control.launch.py
```

Useful for checking controller wiring, joint names and the TF tree in a couple
of seconds rather than waiting for Gazebo. It *does* start a `ros2_control_node`,
which is correct here — there is no Gazebo plugin to host the controller manager.

---

## rviz/docking.rviz

| display | topic | shows |
| --- | --- | --- |
| RobotModel | `/robot_description` | the robot |
| TF | — | all frames |
| MarkerPose (red axes) | `/docking/marker_pose` | the marker as the detector currently sees it |
| DockEstimate (yellow axes) | `/docking/dock_estimate` | the filtered dock pose the controller is steering to, in `odom` |
| PreDockWaypoint (green arrow) | `/docking/predock_pose` | the waypoint the controller is driving at |
| CameraDebug | `/docking/debug_image` | camera view with the marker outlined and its axes drawn |

Fixed frame is `odom`, so the dock stays still and the robot moves.

Two settings that are easy to get wrong when hand-editing this file:

- The RobotModel display's description topic needs **Durability Policy:
  Transient Local**. `/robot_description` is latched, and a Volatile subscriber
  silently misses the one message that was ever published — the robot simply
  never appears.
- Image displays must match the publisher's reliability. `/docking/debug_image`
  is published Reliable; a raw camera topic from Gazebo may be Best Effort.

Comparing **MarkerPose** with **DockEstimate** is the fastest way to see whether
a docking problem is perception or control: if the yellow axes sit still and
match the dock while the red ones jump around, the filter is doing its job and
the problem is downstream.

---

## Adding a launch file

Launch files are installed by the `glob('launch/*.py')` entry in `setup.py`, so
a new file is picked up automatically — but only after a `colcon build`. With
`--symlink-install`, *edits* to an already-installed file are live; a *new* file
still needs the build to create the symlink.
