# perceptron_robot_control

`ros2_control` configuration for the drivetrain. No code — this package is two
YAML files, but they are the files that decide whether the robot drives where
you told it to.

```
config/
  controllers.yaml       simulation (gazebo_ros2_control)
  controllersRviz.yaml   mock hardware, for RViz-only testing
```

---

## What ros2_control is doing here

Worth writing down, because the layering is the part that is easy to forget.

```
  /diff_drive_controller/cmd_vel_unstamped   (geometry_msgs/Twist)
                 |
                 v
      diff_drive_controller          converts Twist -> per-wheel angular velocity
                 |                   and integrates wheel positions -> /odom + TF
                 |   writes "velocity" command interfaces
                 v
        controller_manager            owns the controllers and the hardware
                 |   reads/writes hardware interfaces
                 v
     gazebo_ros2_control/GazeboSystem  applies joint velocities inside gzserver
                 |
                 v
            4 wheel joints in Gazebo

      joint_state_broadcaster         reads the same joints -> /joint_states
                                      -> robot_state_publisher -> TF for the wheels
```

Three pieces of vocabulary:

- **Hardware interface** — the thing that actually talks to motors. Declared in
  the URDF's `<ros2_control>` block, not here. In simulation it is
  `gazebo_ros2_control/GazeboSystem`; with `is_sim:=false` it is
  `mock_components/GenericSystem`, which just echoes commands back as state.
- **Controller** — an algorithm that reads state interfaces and writes command
  interfaces. `diff_drive_controller` and `joint_state_broadcaster` here.
  Configured in this package.
- **Spawner** — `ros2 run controller_manager spawner <name>` loads, configures
  and activates a controller. It is a one-shot process that exits when done; the
  launch files chain them so the broadcaster comes up before the drive
  controller.

**Where the controller_manager lives matters.** With `gazebo_ros2_control` the
controller manager is created *inside gzserver* by the plugin. Do not also start
a `ros2_control_node` — two managers fighting over the same joints is a classic
cause of "controller manager not available" and of controllers that activate but
move nothing. `gazebo_control2.launch.py` deliberately starts no
`ros2_control_node`; `rviz_control.launch.py`, which has no Gazebo, does.

---

## controllers.yaml

Used by the simulation. The path is passed into the xacro as
`gazebo_controllers` and ends up inside the `<plugin>` tag, so the Gazebo plugin
loads it directly.

### Geometry — must match the URDF

```yaml
wheel_separation: 0.37762      # measured track, 2 * wheel_y
wheel_radius: 0.0625           # tyre.stl is 125 mm diameter
wheel_separation_multiplier: 1.16
```

The original values (0.065 and 0.416) were guesses and put roughly 4% of error
on every distance and 10% on every heading — fatal when the docking tolerance is
±2 cm. If you change the URDF's wheel positions or the tyre mesh, change these
in the same commit.

### `wheel_separation_multiplier`, and how to measure it

A 4-wheel skid-steer does not pivot about its geometric track. All four tyres
must scrub sideways to turn, which makes the robot behave as if its wheels were
further apart than they are: the same wheel-speed difference produces *less* yaw
than pure differential-drive kinematics predicts. `diff_drive_controller`
multiplies `wheel_separation` by this factor in both directions — the Twist to
wheel-speed conversion *and* the odometry integration.

Measured in this world, the effective track is 0.437 m against a geometric
0.37762 m, hence **1.16**. With that value, odometry yaw tracks ground truth to
about 0.4%.

Re-measure it whenever tyre friction, robot mass or wheel positions change:

```bash
bash tools/drivetest.sh
```

That script commands a known linear and angular velocity, then compares
`/gazebo/model_states` ground truth against `/diff_drive_controller/odom`. To do
it by hand:

```bash
ros2 topic pub -r 20 /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist '{angular: {z: 0.5}}'
# watch ground truth and odometry yaw diverge
ros2 topic echo /gazebo/model_states
ros2 topic echo /diff_drive_controller/odom
```

`new_multiplier = old_multiplier × (odometry_yaw / true_yaw)`.

Getting this right matters more than it looks: the docking controller's final
`FINAL_CONTACT` push is dead-reckoned on odometry, so odometry that lies about
yaw walks the robot off the dock axis on the last 19 cm.

### The rest

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100            # Hz, the control loop
    use_sim_time: true

diff_drive_controller:
  ros__parameters:
    left_wheel_names:  ["left_front", "left_back"]     # joint names from the URDF
    right_wheel_names: ["right_front", "right_back"]

    base_frame_id: base_footprint
    odom_frame_id: odom
    enable_odom_tf: true        # publishes the odom -> base_footprint transform
    publish_rate: 50.0

    open_loop: false            # integrate measured wheel positions, not commands
    position_feedback: true
    use_stamped_vel: false      # => subscribes to ~/cmd_vel_unstamped
    cmd_vel_timeout: 0.5        # stop if no command arrives for this long

    linear:  {x: {max_velocity: 0.6,  max_acceleration: 1.0}}
    angular: {z: {max_velocity: 1.5,  max_acceleration: 2.0}}
```

Two of these bite people:

- **`use_stamped_vel: false`** is why the topic is
  `/diff_drive_controller/cmd_vel_unstamped` and not `/cmd_vel`. Set it to
  `true` and the controller expects `TwistStamped` on `~/cmd_vel` instead.
- **`cmd_vel_timeout: 0.5`** means a controller publishing at less than 2 Hz
  produces a stuttering robot. The docking controller runs at 20 Hz.

The acceleration limits are also a safety net: they stop a step command from
spinning the wheels hard enough to break traction, which would make odometry
lie.

---

## controllersRviz.yaml

Same controllers, but for `rviz_control.launch.py`, which runs
`mock_components/GenericSystem` instead of Gazebo. `open_loop: true` and
`position_feedback: false` because there is no physics to give feedback from.
Useful for checking that the controller wiring and TF tree are right without
waiting for Gazebo to start.

---

## Checking it works

```bash
ros2 control list_controllers
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# diff_drive_controller[diff_drive_controller/DiffDriveController]       active

ros2 control list_hardware_interfaces
# every wheel joint should show a claimed velocity command interface

ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
```

If `list_controllers` reports "No controllers are currently loaded", the
spawners have not run yet or the controller manager never came up. If the
controller manager never came up in simulation, check the URDF for the
colon-space trap described in
[`perceptron_robot_description/README.md`](../perceptron_robot_description/README.md).

If a controller is `inactive` rather than `active`, the usual cause is a joint
name in this YAML that does not exist in the URDF.
