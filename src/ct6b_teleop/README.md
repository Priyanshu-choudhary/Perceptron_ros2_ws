# ct6b_teleop

ROS 2 Humble teleoperation package for the **FlySky FS-CT6B** 6-channel RC transmitter. It decodes the 18-byte trainer port serial protocol at 115200 baud and publishes `geometry_msgs/msg/Twist` on `/cmd_vel` at **10 Hz**.

Designed as a direct physical remote replacement for `teleop_twist_keyboard` during manual driving for SLAM mapping and navigation benchmarking.

---

## Features

- **10 Hz Control Rate**: Publishes strictly at 10 Hz to prevent flooding or starving downstream motor controllers.
- **Pitch & Roll Mapping**:
  - **Pitch (Channel 2)**: Forward / Backward `linear.x` (Max: 0.5 m/s).
  - **Roll (Channel 1)**: Left / Right rotation `angular.z` (Max: 1.0 rad/s).
- **Deadband Zone**: Center ±40 µs deadzone around 1500 µs neutral to prevent stick jitter / robot creeping.
- **Safety Watchdog**: Automatically zeroes motor velocity if transmitter is powered off, unplugged, or signal drops for > 0.5 s.
- **Graceful Shutdown**: Publishes zero velocity upon node shutdown (`Ctrl+C`).
- **Auto-Detection**: Auto-locates active serial device (`/dev/ttyUSB*`, `/dev/ttyACM*`) if default port changes.

---

## 1. Connecting CT6B to WSL2 (via usbipd)

Since ROS 2 runs in WSL2, the USB-to-serial cable plugged into Windows must be attached to WSL2.

In **PowerShell (Windows)**:
```powershell
# 1. List connected USB devices
usbipd list

# 2. Attach the CP210x / FTDI USB device to WSL (replace 1-2 with your BUSID)
usbipd attach --wsl -b 1-2
```

In **WSL2 Terminal**, verify it appears:
```bash
ls -l /dev/ttyUSB*
```

---

## 2. Build

In your WSL2 workspace (`perceptron_test_ws`):

```bash
cd ~/perceptron_test_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ct6b_teleop
source install/setup.bash
```

---

## 3. Running

### Run Standalone Node
```bash
ros2 run ct6b_teleop ct6b_teleop_node
```

With port override:
```bash
ros2 run ct6b_teleop ct6b_teleop_node --ros-args -p serial_port:=/dev/ttyUSB0
```

### Run via Launch File
```bash
ros2 launch ct6b_teleop ct6b_teleop.launch.py
```

With custom speed limits or port:
```bash
ros2 launch ct6b_teleop ct6b_teleop.launch.py serial_port:=/dev/ttyUSB0 max_linear_speed:=0.5 max_angular_speed:=1.0
```

---

## 4. Verification

In a second WSL terminal:
```bash
# Check publication rate (will show ~10.0 Hz)
ros2 topic hz /cmd_vel

# Check live Twist values
ros2 topic echo /cmd_vel
```

---

## 5. Parameters (`config/ct6b_params.yaml`)

| Parameter | Type | Default | Description |
|---|---|---|---|
| `serial_port` | string | `"/dev/ttyUSB0"` | Device serial path |
| `baud_rate` | int | `115200` | Baud rate for CT6B |
| `cmd_vel_topic` | string | `"/cmd_vel"` | Published topic |
| `publish_rate_hz` | float | `10.0` | Output rate in Hz |
| `max_linear_speed` | float | `0.5` | Max forward/reverse speed (m/s) |
| `max_angular_speed` | float | `1.0` | Max angular turning rate (rad/s) |
| `deadzone` | int | `40` | Deadband radius in µs around center |
| `center_pwm` | int | `1500` | Center PWM value (µs) |
| `min_pwm` | int | `1000` | Min PWM value (µs) |
| `max_pwm` | int | `2000` | Max PWM value (µs) |
| `channel_pitch` | int | `1` | Pitch channel index (0-indexed: 1 = Ch2) |
| `channel_roll` | int | `0` | Roll channel index (0-indexed: 0 = Ch1) |
| `invert_pitch` | bool | `true` | Invert forward/backward direction |
| `invert_roll` | bool | `true` | Invert left/right turn direction |
| `watchdog_timeout` | float | `0.5` | Timeout in seconds before zeroing velocity |
