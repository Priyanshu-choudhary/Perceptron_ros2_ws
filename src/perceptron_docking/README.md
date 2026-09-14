# perceptron_docking

ArUco-marker auto-docking: find the dock, drive to it squarely, stop against it.
Two nodes and a CLI helper.

```
perceptron_docking/
  aruco_detector_node.py     camera -> marker pose in base_footprint
  docking_controller_node.py marker pose -> Twist. The state machine.
  dock_client.py             `ros2 run perceptron_docking dock start|cancel|status`
config/
  docking_params.yaml        every tuning knob
launch/
  docking.launch.py             the two nodes (sim or hardware)
  docking_simulation.launch.py  the above + Gazebo + RViz
```

This package is hardware-agnostic. It publishes a `Twist` and subscribes to an
`Odometry` and a camera; whether those come from Gazebo or from the STM32 bridge
is a launch argument.

---

## Quick start

```bash
ros2 launch perceptron_docking docking_simulation.launch.py

# docking does not start on its own:
ros2 run perceptron_docking dock start
ros2 run perceptron_docking dock cancel
ros2 run perceptron_docking dock status

# or the raw services
ros2 service call /docking/start  std_srvs/srv/Trigger {}
ros2 service call /docking/cancel std_srvs/srv/Trigger {}
```

Watch it:

```bash
ros2 topic echo /docking/status
ros2 run rqt_image_view rqt_image_view      # pick /docking/debug_image
```

---

## Interface

| direction | name | type |
| --- | --- | --- |
| in | `/camera/image_raw`, `/camera/camera_info` | `sensor_msgs/Image`, `CameraInfo` |
| in | `/docking/marker_pose` | `geometry_msgs/PoseStamped` (detector → controller) |
| in | `/diff_drive_controller/odom` or `/odom` | `nav_msgs/Odometry` |
| out | `/docking/marker_pose` | pose of the marker **in `base_footprint`** |
| out | `/docking/marker_detected` | `std_msgs/Bool`, one per frame |
| out | `/docking/debug_image` | annotated camera image |
| out | `/docking/status` | `std_msgs/String`, state name at 20 Hz |
| out | `/docking/dock_estimate` | filtered dock pose in `odom` |
| out | `/docking/predock_pose` | the waypoint being driven to, in `base_footprint` |
| out | `/diff_drive_controller/cmd_vel_unstamped` or `/cmd_vel` | `geometry_msgs/Twist` |
| srv | `/docking/start`, `/docking/cancel` | `std_srvs/Trigger` |
| tf | `camera_optical_link -> aruco_dock_marker_<id>` | broadcast per detection |

---

## aruco_detector_node

Per camera frame:

1. Convert to grayscale, run `cv2.aruco.detectMarkers`.
2. Keep the marker whose id equals `marker_id`; ignore the rest.
3. `cv2.solvePnP` with `SOLVEPNP_IPPE_SQUARE` on the four corners against a
   square of side `marker_size`, giving the pose in the **camera optical frame**.
4. Broadcast that as TF, then transform it into `base_frame` and publish it on
   `/docking/marker_pose`.
5. Publish the annotated debug image.

### Things worth remembering

**The pose is published in `base_footprint`, not the camera frame.** That is
what lets the controller work in plain robot coordinates (x forward, y left). If
the TF lookup fails the node publishes **nothing** and warns. That is
deliberate: an optical-frame pose looks like a perfectly valid message but has
x = right and z = forward, so a controller reading it as x-forward drives
sideways into the dock. Silence is safer than a wrong frame.

**Marker +Z is the outward normal of the printed face.** With OpenCV's corner
order (top-left, top-right, bottom-right, bottom-left) the marker frame's +Z
points back out of the marker towards whoever is looking at it. The controller
uses that to work out which way the dock faces — which is what makes a square
approach possible.

**`marker_size` is the side of the black square**, not the paper or the plate it
is printed on. `solvePnP` scales distance linearly with it, so a 4% error here is
a 4% error on every range.

**Camera intrinsics come from `/camera/camera_info`** when `use_camera_info` is
true. The `fallback_camera_matrix` is only used when it is false, and the values
in the YAML are the *simulated* camera's — they will be wrong for a real one.
Calibrate with `ros2 run camera_calibration cameracalibrator`.

**Best-effort QoS on the image subscription.** A Best Effort subscriber matches
both Best Effort and Reliable publishers; a Reliable subscriber will silently
never connect to a Best Effort publisher. Since camera drivers differ, Best
Effort is the safe choice. "Topic exists, callback never fires" is nearly always
a QoS mismatch — check with `ros2 topic info /camera/image_raw --verbose`.

### Parameters

| parameter | default | notes |
| --- | --- | --- |
| `marker_id` | 42 | Only this id is acted on. |
| `marker_size` | 0.15 | Metres, side of the black square. Must match the physical marker. |
| `dictionary_name` | `DICT_5X5_250` | Must match how the marker was generated. |
| `camera_image_topic` / `camera_info_topic` | `/camera/image_raw`, `/camera/camera_info` | |
| `use_camera_info` | `true` | False falls back to `fallback_camera_matrix`. |
| `camera_optical_frame` | `camera_optical_link` | Only used if the image header has no `frame_id`. |
| `base_frame` | `base_footprint` | Frame the pose is published in. |
| `publish_debug_image` | `true` | Costs a full image copy per frame. |

---

## docking_controller_node

```
IDLE --start--> SEARCHING --marker--> APPROACH --at pre-dock--> FINAL_APPROACH
                    ^                    |                            |
                    +---- LOST_RECOVERY -+                     FINAL_CONTACT
                                                                      |
                                                                    DOCKED
```

| state | behaviour | leaves when |
| --- | --- | --- |
| `IDLE` | Nothing. | `/docking/start` |
| `SEARCHING` | Rotate on the spot at `search_angular_velocity`. | a marker is seen |
| `APPROACH` | Curve onto the pre-dock pose, then turn on the spot to square up. | within `predock_position_tolerance` and pointing at the dock, with live vision |
| `FINAL_APPROACH` | Follow the dock axis inwards to `target_docking_distance`. | all three tolerances met |
| `FINAL_CONTACT` | Drive straight, blind, for `final_push_distance` on odometry. | distance reached, or stalled for `contact_timeout_seconds` |
| `DOCKED` | Stopped. | `/docking/start` again |
| `LOST_RECOVERY` | Stop and wait. | marker returns, or falls back to `SEARCHING` |
| `FAILED` | Stopped after `max_docking_time_seconds`. | `/docking/start` again |

### The four ideas that make it work

Each of these was a failure first; the debugging story is in
[`SIMULATION.md`](../../SIMULATION.md).

**1. The dock is tracked in `odom`, not in the camera frame.** Every detection
is converted into `odom` and blended into a running estimate; the control loop
converts it back into robot coordinates each tick. Two consequences:

- The dock does not move, so filtering in `odom` is free noise rejection **with
  no lag** — unlike filtering in the robot frame, where the target legitimately
  moves as the robot drives and a filter adds delay.
- The robot can turn away from the marker mid-manoeuvre without losing its
  target. Tracking only what the camera sees right now makes the robot
  oscillate between APPROACH and SEARCHING forever on any approach that is not
  already head-on, because turning towards the pre-dock waypoint necessarily
  points the camera away from the dock.

Published on `/docking/dock_estimate` so you can watch it in RViz.

**2. APPROACH drives to a pre-dock pose, not at the marker.** The waypoint sits
`predock_standoff` out along the marker's own normal, facing the marker:

```
        dock                        pre-dock waypoint
      ===|===  <----- normal ----->      (o)-->
         |          0.75 m
```

A rho/alpha/beta pose regulator curves the robot onto it:

```
rho   = distance to the waypoint
alpha = bearing to the waypoint, in the robot frame
beta  = desired final heading - alpha

v = k_rho * rho
w = k_alpha * alpha + k_beta * beta      stable for k_rho > 0, k_beta < 0, k_alpha > k_rho
```

Homing straight at the marker instead arrives at the dock at an angle and wedges
the robot against the plate. Once the robot is *standing on* the waypoint it
switches to turning on the spot, because at rho near zero `alpha` is the `atan2`
of two noisy near-zero numbers and the law thrashes — the robot orbits the
waypoint instead of settling on it.

**3. FINAL_APPROACH follows a line, it does not home on a point.** A
differential drive cannot remove a lateral offset by turning on the spot, so the
cross-track error has to be steered out *while driving*. Working in dock-axis
coordinates:

```
d_axis = distance to the marker along its normal
e_ct   = cross-track error, positive when the dock axis is to the robot's left
e_head = heading error relative to facing the dock

v = clamp(0.5 * (d_axis - target_docking_distance), 0.025, final_linear_velocity)
w = kp_final_heading * e_head + kp_final_lateral * e_ct
```

Linearised about the axis:

```
e_ct'   =  e_head
e_head' = -(kp_final_heading / v) e_head - (kp_final_lateral / v) e_ct
```

which is stable for any positive gains — but stability is not the point, the
*rate* is. The defaults give a cross-track decay length of about 0.15 m, which
clears a 5 cm offset over the 0.35 m run-in from the pre-dock pose. Gentler gains
look perfectly stable and simply never converge: the robot settles at a fixed
offset with a compensating heading and shuttles in and out forever. If it does
reach the stop distance still off the axis, it reverses and runs the approach
again.

**4. FINAL_CONTACT is deliberately blind.** A 0.15 m marker fills the camera's
vertical field of view once the lens is within 0.119 m of it, and the camera sits
0.183 m ahead of `base_footprint`, so the marker is gone before the robot is
docked. The controller stops the visual servo at `target_docking_distance`
(0.40 m) and dead-reckons the last `final_push_distance` (0.19 m) on wheel
odometry — exactly what the real robot has to do. If it stalls against the dock
for `contact_timeout_seconds` that counts as docked too.

This is why `wheel_separation_multiplier` in `perceptron_robot_control` has to be
measured rather than guessed: odometry that lies about yaw walks the robot off
the dock axis during the blind push.

### Parameters

`config/docking_params.yaml`. The geometry values encode *this* robot; re-derive
them if the camera or bumper moves.

#### Manoeuvre geometry

| parameter | default | meaning |
| --- | --- | --- |
| `predock_standoff` | 0.75 | Waypoint distance out along the marker normal. Larger gives a gentler curve and a longer run-in to straighten out; smaller docks faster but with less room to correct. |
| `predock_position_tolerance` | 0.08 | How close counts as "at the waypoint". Tighter than the robot can hold and it never leaves APPROACH; slacker and FINAL_APPROACH starts with more cross-track to remove. |
| `target_docking_distance` | 0.40 | `base_footprint` to marker where the visual servo stops. Must stay above the field-of-view limit. |
| `final_push_distance` | 0.19 | Blind odometry push. Nominally `0.40 − 0.19 = 0.21 m` final range against a bumper 0.196 m ahead of `base_footprint`, so ~1.4 cm clearance; measured runs land at 2.8–3.0 cm because the visual stop is allowed ±`distance_tolerance`. |
| `contact_timeout_seconds` | 8.0 | If the push stalls this long (robot against the dock, wheels slipping), call it docked. |

#### Tolerances — these set the final accuracy

| parameter | default | meaning |
| --- | --- | --- |
| `distance_tolerance` | 0.02 | Along-axis error accepted at the stop point. |
| `lateral_tolerance` | 0.025 | Cross-track error accepted. |
| `angle_tolerance` | 0.07 | ~4°. Residual heading also drags the robot sideways during the blind push: 0.07 rad over 0.19 m is about 1.3 cm. |

Tightening these below what the perception can actually resolve makes the robot
shuttle back and forth instead of docking.

#### Velocities

| parameter | default |
| --- | --- |
| `max_linear_velocity` / `min_linear_velocity` | 0.25 / 0.04 |
| `max_angular_velocity` | 0.60 |
| `search_angular_velocity` | 0.35 |
| `final_linear_velocity` | 0.07 |
| `contact_linear_velocity` | 0.05 |

`final_linear_velocity` is coupled to the FINAL_APPROACH gains — change it and
re-derive them (below).

#### Gains

| parameter | default | notes |
| --- | --- | --- |
| `k_rho`, `k_alpha`, `k_beta` | 0.55, 1.40, −0.55 | APPROACH pose regulator. |
| `kp_final_heading` | 0.95 | FINAL_APPROACH heading gain. |
| `kp_final_lateral` | 3.10 | FINAL_APPROACH cross-track gain. **This is the one that pulls the robot onto the axis** — raising only the heading gain makes it hold a steady offset. |
| `pose_filter_alpha` | 0.3 | Blend factor for the odom-frame dock estimate. Lower is smoother; costs no lag because the dock is static. |

To retune FINAL_APPROACH for a different speed `v` and desired cross-track decay
length `L` (critically damped):

```
kp_final_heading = 2 * v / L
kp_final_lateral = v / L^2
```

Defaults are `v = 0.07`, `L ≈ 0.15`.

#### Watchdogs and topics

| parameter | default | meaning |
| --- | --- | --- |
| `marker_timeout_seconds` | 1.5 | Beyond this a detection is no longer "live vision". FINAL_APPROACH requires live vision. |
| `estimate_timeout_seconds` | 10.0 | How long the odom-propagated estimate stays usable with no detection at all. Past this the controller gives up and re-searches. |
| `max_docking_time_seconds` | 180.0 | Whole-sequence abort into `FAILED`. |
| `cmd_vel_topic` | `/diff_drive_controller/cmd_vel_unstamped` | `/cmd_vel` on the real robot. |
| `odom_topic` | `/diff_drive_controller/odom` | `/odom` on the real robot. |
| `auto_start` | `false` | `true` starts searching as soon as the node comes up. |

---

## Launch files

### docking.launch.py — the two nodes only

| argument | default |
| --- | --- |
| `config_file` | installed `docking_params.yaml` |
| `auto_start` | `false` |
| `cmd_vel_topic` | `/diff_drive_controller/cmd_vel_unstamped` |
| `odom_topic` | `/diff_drive_controller/odom` |
| `use_sim_time` | `true` |

```bash
# real robot
ros2 launch perceptron_docking docking.launch.py \
     use_sim_time:=false cmd_vel_topic:=/cmd_vel odom_topic:=/odom
```

`use_sim_time` is not optional in simulation. Without it these nodes stamp
messages with wall time while TF runs on sim time, and every
`lookup_transform` fails with an extrapolation error. The controller uses the
node clock throughout for the same reason.

### docking_simulation.launch.py — everything

Includes `gazebo_control2.launch.py`, then starts the docking pipeline behind an
8 s `TimerAction` so the camera is publishing before the detector comes up, then
RViz.

| argument | default |
| --- | --- |
| `gui`, `rviz` | `true`, `true` |
| `simple_visuals`, `software_gl` | `false`, `false` |
| `auto_start` | `false` |
| `x`, `y`, `yaw` | `0`, `0`, `0` — spawn pose, handy for testing odd approaches |

---

## Tuning, in order

Do not touch gains before checking the geometry. Most "the controller is bad"
problems are a wrong number somewhere upstream.

1. **Is the marker detected at all?** `/docking/debug_image` and
   `ros2 topic echo /docking/marker_detected`. If not, it is size, distance,
   dictionary or lighting — see
   [`perceptron_robot_gazebo/README.md`](../perceptron_robot_gazebo/README.md).
2. **Is the range right?** Compare `/docking/marker_pose` with ground truth:
   `bash tools/rangetest.sh 1.55`. A constant percentage error means
   `marker_size` does not match the physical marker.
3. **Is odometry right?** `bash tools/drivetest.sh`. The blind final
   push depends on it.
4. **Then** the approach gains, watching `/docking/status` and RViz.

**Every parameter is read once, in the node's constructor.** There is no
`add_on_set_parameters_callback`, so `ros2 param set` changes the parameter
server and nothing else — the running controller keeps the value it started
with. To retune, edit `config/docking_params.yaml` and relaunch just the two
docking nodes:

```bash
ros2 launch perceptron_docking docking.launch.py     # Gazebo can keep running
```

With `--symlink-install` the installed YAML is a symlink to the source file, so
no rebuild is needed. (Wiring up a parameter callback so gains *are* live would
be a small change and a real convenience if you end up tuning often.)

### Symptoms

| symptom | likely cause |
| --- | --- |
| Cycles SEARCHING → APPROACH → SEARCHING | Marker not detectable from the approach path (too small, too oblique), or `estimate_timeout_seconds` too short. |
| Orbits the pre-dock waypoint | `predock_position_tolerance` too tight. |
| Shuttles in and out near the dock | Cross-track gain too low, or tolerances tighter than the perception noise. |
| Docks crooked | `angle_tolerance` too slack, or the run-in from the pre-dock pose too short to straighten out — raise `predock_standoff`. |
| Stops short of the dock | `final_push_distance` too small, or odometry under-reporting distance. |
| Drives into the dock and pushes | `final_push_distance` too large. Harmless in sim; check `contact_timeout_seconds` catches it. |

---

## Known limits

- The controller assumes the robot starts roughly in front of the dock, close
  enough that rotating on the spot brings the marker into view. It does not
  navigate — getting into that region is Nav2's job. Starting behind the dock
  post, where the marker is occluded from every heading, searches until
  `max_docking_time_seconds` and then reports `FAILED`.
- `SEARCHING` only rotates. There is no translating search pattern.
- Only one marker id is tracked. Multi-dock would need an id-to-dock map.
- No obstacle avoidance during the approach.
