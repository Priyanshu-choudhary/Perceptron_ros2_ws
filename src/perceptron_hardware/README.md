# perceptron_hardware

Serial bridge between ROS 2 and the STM32 motor controller. This is what makes
the real robot look like the simulation to everything above it: `Twist` in,
`Odometry` and TF out.

```
perceptron_hardware/
  stm32_bridge_node.py
config/
  hardware_params.yaml
launch/
  hardware.launch.py     robot_state_publisher (is_sim:=false) + the bridge
```

The STM32 firmware itself lives outside this workspace, in
`../STM32_Serial_motorDriver`.

---

## Running

```bash
ros2 launch perceptron_hardware hardware.launch.py serial_port:=/dev/ttyUSB0
```

Then the same autonomy as in simulation, pointed at the hardware topics:

```bash
ros2 launch perceptron_docking docking.launch.py \
     use_sim_time:=false cmd_vel_topic:=/cmd_vel odom_topic:=/odom
```

Note `hardware.launch.py` runs `robot_state_publisher` with `is_sim:=false`, so
the URDF uses `mock_components/GenericSystem` and carries no Gazebo tags. It
does **not** start a controller manager: on the real robot the STM32 closes the
wheel loops itself, and ROS only speaks `Twist` and `Odometry`.

---

## Interface

| direction | name | type | notes |
| --- | --- | --- | --- |
| in | `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` and `angular.z` only |
| out | `/odom` | `nav_msgs/Odometry` | twist only; pose is left unset and marked unmeasured |
| out | `/imu/data_raw` | `sensor_msgs/Imu` | |

---

## Serial protocol

115200 baud 8N1 by default. Two directions, three frame types, XOR checksums.

### Host to STM32

```
 0xA5 | type | len | payload[len] | xor | 0x5A
```

| type | name | payload |
| --- | --- | --- |
| `0x01` | `CMD_VEL` | `<ff` — linear.x, angular.z, both float32 m/s and rad/s |

The XOR is over `type`, `len` and the payload.

### STM32 to host

Fixed-length frames, header byte then payload then XOR then `0x0A`:

| header | size | `struct` format | fields |
| --- | --- | --- | --- |
| `0x55` | 27 B | `<BIHhiiiiBB` | header, timestamp, bus voltage, current, enc_l, enc_r, vel_l (mm/s), vel_r (mm/s), xor, end |
| `0x56` | 31 B | `<BIffffffBB` | header, timestamp, ax, ay, az, gx, gy, gz, xor, end |

The receive loop keeps a rolling buffer and slides one byte at a time on a bad
checksum or a missing end byte, so it resynchronises after line noise instead of
locking up.

The ECU reports wheel velocities only. Pose is estimated on the host by
robot_localization, which fuses this twist with the IMU.
The bridge only derives the twist from the reported wheel velocities.

---

## Parameters (`config/hardware_params.yaml`)

| parameter | default | meaning |
| --- | --- | --- |
| `serial_port` | `/dev/ttyUSB0` | Overridden by the `serial_port` launch argument. |
| `baud_rate` | 115200 | Must match the firmware. |
| `timeout` | 0.01 | pyserial read timeout, seconds. |
| `base_frame_id` | `base_footprint` | Odometry child frame. |
| `odom_frame_id` | `odom` | |
| `imu_frame_id` | `imu_link` | |
| `publish_tf` | `false` | Retained for launch compatibility; the bridge has no pose to broadcast, so setting it only logs a warning. |
| `cmd_vel_timeout` | 0.3 | Watchdog: after this long with no `/cmd_vel`, send a zero-velocity stop. |

The watchdog is the important one. If the docking controller crashes mid-approach
the robot stops within 300 ms instead of continuing into the dock at whatever it
was last told.

---

## Node behaviour

- **Reconnects on its own.** If the port is missing at startup or disappears, the
  node warns and retries roughly every 2 s rather than dying.
- **100 Hz serial poll**, 20 Hz watchdog.
- **Wall clock, not the node clock.** Correct here — there is no simulated time
  on hardware — but it means `use_sim_time` must be **false** for everything in
  a hardware session.

---


---

## Battery monitor

`battery_node` publishes the pack state that the docking and recharge logic
decides on. Values are simulated for now; every parameter is live.

| topic | type | meaning |
| --- | --- | --- |
| `/battery/state` | `sensor_msgs/BatteryState` | voltage, current, charge, percentage, status, health |
| `/battery/power` | `std_msgs/Float32` | watts |
| `/battery/low` | `std_msgs/Bool` | charge below `low_battery_percentage` — head for the dock |
| `/battery/critical` | `std_msgs/Bool` | nearly flat, or cells sagging below their floor — stop |

```bash
ros2 topic echo /battery/state
ros2 param set /battery_node voltage 11.1     # ~23%, trips /battery/low
ros2 param set /battery_node voltage 10.5     # ~4%,  trips /battery/critical
ros2 param set /battery_node current -8.0     # heavier load, more sag
```

Configuration is `config/battery_params.yaml`. It runs automatically in both
`gazebo_control2.launch.py` (`use_battery:=false` to disable) and
`hardware.launch.py`.

### Why there is a separate power topic

`sensor_msgs/BatteryState` carries voltage, current, charge, capacity and
percentage — but has **no power field**. Power is voltage times current, so it
is published separately rather than smuggled into a field that means something
else.

### Current sign

ROS convention: current is **negative while discharging**, positive while
charging. A node reporting `+2.5 A` on a robot that is driving around is telling
every consumer the pack is filling up. `power_supply_status` is derived from the
sign, so getting it wrong makes the robot think it is charging as it flattens.

### Charge from voltage, and why percentage is the threshold

LiPo voltage maps to charge through a curve that is famously flat in the middle
and then falls off a cliff. Linear interpolation between empty and full is wrong
by tens of percent around the middle, so the node uses a standard resting-voltage
table.

That flatness is also why **the "head for the dock" threshold is a percentage,
not a voltage**. Volts per cell reads like the natural unit and is a trap:

| volts per cell | 3S pack | actual charge |
| --- | --- | --- |
| 3.85 | 11.55 V | 55% |
| 3.80 | 11.40 V | 40% |
| 3.75 | 11.25 V | 25% |
| 3.70 | 11.10 V | 12% |
| 3.50 | 10.50 V | **3%** |

A threshold of 3.50 V/cell looks like a comfortable margin and means "you have
3% left". `low_battery_percentage: 0.25` says what it means.

Cell **protection** is the opposite case and does use voltage — the measured
terminal voltage, sag included, because sag under load is what damages cells.
Compensating the sag away first would let the pack be dragged under its floor
while the estimate still looked healthy. Hence two signals from two different
quantities:

```
/battery/low       <- state of charge          (a planning decision)
/battery/critical  <- measured terminal volts  (a protection decision)
```

### Load sag

Voltage drops under current draw, so charge read straight off a loaded pack
reads low. The node undoes this with an ohmic model,
`V_open = V_measured + |I| * internal_resistance`, before consulting the curve.
That compensation is only as good as `internal_resistance`, which varies with
temperature, age and chemistry. For anything safety-critical, count coulombs
instead of trusting voltage.

### Per-cell voltages

`cell_voltage` is filled with an even split of the pack voltage. There is no
per-cell sensing without a BMS, and real balance data would not be identical
across cells like this. Treat it as a placeholder, not as balance information.

### Connecting the real pack

**The data already arrives and is currently thrown away.** The STM32's `0x55`
telemetry frame carries bus voltage and current; `stm32_bridge_node._parse_telemetry`
unpacks them as `bus_raw` and `cur_raw` and then ignores both:

```python
_, ts, bus_raw, cur_raw, enc_l, enc_r, vel_l_mms, vel_r_mms, _, _ = struct.unpack(...)
```

To go live: publish those two from the bridge (applying whatever scaling the
firmware uses to pack them into a `uint16` and an `int16`), have `battery_node`
subscribe instead of reading its `voltage` and `current` parameters, and set
`simulate: false`. Check the firmware for the scaling factors before trusting
the numbers — a raw count is not millivolts until you have confirmed it is.

### Using it for docking decisions

The signals exist; nothing consumes them yet. The natural integration is in
`mission_node`: subscribe to `/battery/low`, and when it goes true mid-patrol,
abandon the remaining waypoints and go straight to `_dock()`. `/battery/critical`
should stop the robot where it stands rather than start a journey it cannot
finish. Note that `charge_when_docked` already flips `power_supply_status` to
CHARGING when `/docking/status` reports DOCKED, so a full patrol-charge-resume
cycle is testable end to end in simulation.

## Known discrepancies

Two things in this package do not currently agree with the rest of the
workspace. Neither is dangerous, but both will confuse you later.

**1. Hardcoded wheel separation of 0.416 m.** In `_parse_telemetry`:

```python
v_angular = ((vel_r_mms - vel_l_mms) / 1000.0) / 0.416
```

The measured track is **0.37762 m** (see
[`perceptron_robot_control/README.md`](../perceptron_robot_control/README.md)),
so the reported `twist.angular.z` is about 10% low. It affects only the *twist*
field, which is now the only rotation this message carries. Since the EKF fuses
`vyaw` from `/odom` whenever `use_imu` is false, the value matters directly in
that mode. A skid-steer also wants the *effective* track, not the geometric one,
so 0.437 m is probably the right number rather than 0.37762.

**2. `publish_rate_hz: 50.0` in the YAML is never declared or used** by the node;
publish rate is driven by how fast the STM32 sends telemetry. Harmless, but it
looks like a knob and is not one.

---

## Bringing up a real robot

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*              # find the port
sudo usermod -aG dialout $USER               # then log out and back in

ros2 launch perceptron_hardware hardware.launch.py serial_port:=/dev/ttyACM0
ros2 topic echo /odom                        # is telemetry arriving?

# wheels off the ground for this one
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
```

Checklist before trusting docking on hardware:

1. `/odom` twist updates and the EKF's heading matches reality over a measured 360° turn.
2. Camera publishes `/camera/image_raw` **and** a calibrated `/camera/camera_info`.
3. The printed marker's black square measures exactly `marker_size`.
4. `use_sim_time:=false` everywhere: `ros2 param get <node> use_sim_time`.
5. Reset the EKF pose (`/set_pose`) before each docking run, so the odom frame starts clean.
