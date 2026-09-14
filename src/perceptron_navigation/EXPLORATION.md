# Explore unknown rooms and search for ArUco markers

This mode coordinates laser SLAM, frontier goals, Nav2, and camera inspection.
It does not require a saved map or a known marker position. The robot must start
in free space with working LiDAR, camera calibration, and map-to-base TF.

## Start the stack

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch perceptron_navigation nav_simulation.launch.py \
  slam:=true exploration:=true \
  marker_dictionary:=DICT_5X5_250 marker_size:=0.15
```

`exploration:=true` selects exclusive exploration mode and suppresses the patrol
and docking nodes, even if their launch flags have their usual true defaults.
The separate search detector observes all IDs in the selected dictionary; it
does not change the docking detector's ID. No motion starts until a command below.
Do not send competing RViz/teleop/navigation goals while exploration is running.

The launch retains your current world/spawn defaults. Override `world`, `x`, `y`,
and `yaw` for another world. The world must actually contain a visible printed
marker with the selected dictionary and size. A numeric ID alone is not enough
to identify its dictionary. The stock TurtleBot3 world does not add an ArUco
marker automatically. Your room-world dock uses ID 42, DICT_5X5_250, 0.15 m.

To use that existing marker as a first example, launch with:

```bash
ros2 launch perceptron_navigation nav_simulation.launch.py \
  world:=$HOME/ros2_ws/src/perceptron_robot_gazebo/worlds/room_world.world \
  x:=-3.0 y:=-2.0 slam:=true exploration:=true
# Then, in another sourced terminal:
ros2 run perceptron_navigation explore search 42
```

## Commands (second sourced terminal)

```bash
# Explore reachable unknown space, updating the SLAM map:
ros2 run perceptron_navigation explore start

# Or explore/search for ID 10, stopping when its map pose is confirmed:
ros2 run perceptron_navigation explore search 10

# Watch without affecting motion:
ros2 run perceptron_navigation explore status

# Cancel the active run:
ros2 run perceptron_navigation explore cancel
```

Use one start/search command at a time. Ctrl-C in a start/search client requests
cancellation. Ctrl-C in a status-only client only stops watching. Closing or
killing a client without allowing cancellation does not guarantee a robot stop;
use `explore cancel`. The coordinator rejects target changes while a run is active.

To select MPPI, launch with `nav_profile:=mppi`. Exploration uses Smac2D + MPPI
in that profile and NavFn + DWB in `dwb`. Goals remain in observed free space;
there is no need to globally enable planning through unknown cells.

For automatic start after the stack becomes ready:

```bash
ros2 launch perceptron_navigation nav_simulation.launch.py \
  slam:=true exploration:=true exploration_auto_start:=true search_marker_id:=10
```

Omit `search_marker_id` (default -1) for automatic mapping without marker search.
The implementation currently requires planar laser SLAM (`lidar_type:=2d` or
`both`, `global_ekf:=false`). Camera-only SLAM and rough-terrain exploration are
separate future features.

## How targets and motion work

1. Wait for fresh map/scan/TF, active Nav2 lifecycle nodes, and the velocity relay.
   Marker searches also require calibrated camera frames and the Spin action.
2. Inflate occupied AND unknown space by the robot clearance requirement.
3. Find free cells next to unknown cells, group them into frontiers, and choose a
   reachable free-space viewpoint behind a frontier. A four-connected flood fill
   rejects disconnected rooms and diagonal corner shortcuts. Nav2 still checks
   dynamic obstacles and plans the actual route using its costmaps.
4. Navigate and let SLAM reveal additional space; retry elsewhere after a failed
   or timed-out goal. Timeouts wait for action cancellation before another goal.
5. During marker search, perform a full camera sweep at the start and after each
   successful arrival using four collision-checked Nav2 quarter-turn actions.
6. Once frontiers are exhausted, inspect additional reachable room viewpoints.
   A room visible to the laser from a doorway may still need a closer camera view.
7. Accept only the requested ID/dictionary, with three distinct fresh, spatially
   consistent map-frame observations. Cancel current motion and save the result.

Search finds and records a marker; it does not automatically approach or dock
against an arbitrary marker. It never interprets ID 10 as docking marker 42.

## Results and limits

Each terminal run saves a timestamped directory under
`~/.ros/perceptron_exploration/` containing:

- `map.pgm` and `map.yaml`: the current SLAM occupancy snapshot, loadable by AMCL.
- `result.json`: requested ID, outcome, map-frame marker position/orientation if
  found, dictionary/size, inspected viewpoints, goal counts, and elapsed time.

Change the root with `exploration_output:=/absolute/path`. Marker coordinates
belong to the saved map snapshot; do not mix them with another map's origin.
The saved occupancy grid does not include a resumable SLAM pose graph.

Topics: `/exploration/status` (JSON String), `/exploration/goal` (PoseStamped),
`/exploration/detections` (MarkerArray with per-observation IDs/optical-frame poses),
`/exploration/found_marker` (map-frame PoseStamped). `result.json` pairs the pose
with its ID; the PoseStamped topic alone does not carry an ID.

Outcomes:

- `FOUND`: requested marker confirmed and action motion stopped.
- `COMPLETE`: no further reachable untried frontier viewpoints across three map updates.
- `NOT_FOUND`: requested marker not observed from reachable sampled viewpoints.
- `EXHAUSTED`: time/goal budget reached, or remaining targets include failed motions.
- `FAILED`: required sensor/TF/lifecycle data lost, invalid starting clearance, etc.
- `CANCELLED`: requested cancellation completed.
- `STOP_UNCONFIRMED`: Nav2 did not acknowledge stopping. Relay disable is requested;
  resolve the outstanding action and relay state before restarting the coordinator.

COMPLETE/NOT_FOUND do not claim that inaccessible rooms, closed doors, occluded
markers, tiny unresolved frontiers, or every possible camera viewpoint were covered.
There is no battery-return behavior in this feature; maximum duration and goal
budgets bound the run. Final marker pose accuracy depends on calibration, marker
size, viewing angle and SLAM accuracy.

## Tuning

`config/exploration.yaml` contains clearance, frontier standoff, camera-viewpoint
spacing, sensor watchdogs, goal/time budgets, and marker confirmation thresholds.
Pass an edited copy with `exploration_config:=/absolute/path/config.yaml` and
relaunch. Only `target_marker_id` is changeable while idle through the CLI; the
other parameters are startup settings. Keep frontier_standoff above the robot
radius plus clearance. Lower viewpoint_spacing for a denser camera search.

## Validation

Offline unit and synthetic ROS integration tests exercise reachability, door
clearance, map expansion/origin, multiple marker IDs, map serialization and action
cancellation races. No Gazebo or real robot movement is needed for these tests.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select perceptron_navigation
source install/setup.bash
python3 -m pytest -q src/perceptron_navigation/test/test_exploration.py
# Optional synthetic ROS test graph, isolated on domain 187; no wheel commands:
python3 -m pytest -q src/perceptron_navigation/test/test_exploration_ros.py
```

Before calling autonomous room search validated, run a Gazebo scenario with an
initially unseen marker in a second connected room, then repeat with an absent
ID, a blocked doorway, a stale sensor, and cancellation during navigation/spin.
Measure map coverage and marker-position error against simulation ground truth.
