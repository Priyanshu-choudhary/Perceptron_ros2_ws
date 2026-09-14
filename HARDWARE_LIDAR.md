# Testing the physical D500 lidar under WSL2

The D500 kit is an **LDROBOT STL-19P**: 360°, 0.03–12 m, ~10 Hz, speaking the
**LD19 protocol at 230400 baud** over a USB-to-serial adapter.

WSL2 has no USB stack of its own. A serial device plugged into Windows is
invisible to Linux until you forward it over USB/IP. That is the only real
obstacle; everything after it is ordinary ROS 2.

Your machine has already been checked and is in good shape:

| | |
|---|---|
| WSL distro | Ubuntu 22.04, ROS 2 Humble |
| WSL kernel | 6.6.87.2-microsoft-standard-WSL2 |
| `cp210x` / `ch341` / `cdc-acm` | present as modules — **no kernel rebuild needed** |
| `usbip` kernel modules | present |
| `usbipd-win` on Windows | **not installed — this is the one thing missing** |

Older guides tell you to recompile the WSL kernel to get USB-serial drivers.
That advice is stale for this kernel. Skip it.

---

## Step 1 — Install usbipd-win (Windows, once)

In an **Administrator** PowerShell:

```powershell
winget install --interactive --exact dorssel.usbipd-win
```

Close and reopen PowerShell afterwards so `usbipd` is on PATH.

## Step 2 — Attach the lidar to WSL (Windows, every time you replug)

Plug the lidar in, then:

```powershell
usbipd list
```

Look for a `Silicon Labs CP210x` or `USB-SERIAL CH340` line and note its BUSID
(e.g. `2-3`). Bind it once, then attach:

```powershell
usbipd bind --busid 2-3
usbipd attach --wsl --busid 2-3
```

`bind` is persistent; `attach` is not — you repeat it after every unplug or WSL
restart. To make it automatic, add `--auto-attach` to the attach command and
leave that terminal open.

## Step 3 — WSL-side setup (once)

```bash
cd ~/perceptron_test_ws
bash tools/setup_d500_wsl.sh
```

That loads the USB-serial modules, installs a udev rule giving you a stable
`/dev/ldlidar` symlink regardless of which `ttyUSB` number it enumerates as, and
clones the LDROBOT driver into `src/`.

Log out and back in (`exit`, then reopen WSL) so the `dialout` group membership
takes effect.

## Step 4 — Confirm the OS sees it

```bash
lsusb
ls -l /dev/ldlidar
dmesg | tail -20
```

You want `lsusb` to show the adapter and `/dev/ldlidar` to point at a
`ttyUSB*`. If `lsusb` shows it but there is no tty, the driver module did not
bind — run `sudo modprobe cp210x` (or `ch341`) and re-check.

### Optional: prove it works without ROS at all

Worth doing once, because it separates "USB passthrough is broken" from "my ROS
setup is wrong":

```bash
stty -F /dev/ldlidar 230400 raw -echo
timeout 2 xxd -l 256 /dev/ldlidar
```

A spinning D500 emits a continuous stream with `54 2c` frame headers. Bytes on
screen means the hardware path is fine and any later problem is in ROS.

## Step 5 — Build and run

```bash
cd ~/perceptron_test_ws
colcon build --symlink-install --packages-select ldlidar_stl_ros2 perceptron_robot_bringup
source install/setup.bash
ros2 launch perceptron_robot_bringup lidar_real.launch.py
```

RViz opens with the fixed frame on `laser_link` and a `/scan` display. You
should see a ring of points redrawing ~10 times a second. Wave your hand in
front of the sensor — the arc should follow it.

Useful arguments:

```bash
# no RViz, just the driver (headless, or over ssh)
ros2 launch perceptron_robot_bringup lidar_real.launch.py rviz:=false

# explicit port if you skipped the udev rule
ros2 launch perceptron_robot_bringup lidar_real.launch.py port:=/dev/ttyUSB0

# full robot bringup is already publishing base_link -> laser_link
ros2 launch perceptron_robot_bringup lidar_real.launch.py publish_tf:=false
```

## Step 6 — Check the data, not just the picture

```bash
ros2 topic hz /scan          # expect ~10 Hz, steady
ros2 topic echo /scan --once # check range_min/range_max, frame_id, NaN count
ros2 run tf2_ros tf2_echo base_link laser_link
```

`ros2 topic hz` is the important one. A rate that sits at 10 Hz and never
stutters means USB/IP is keeping up. A rate that sags or drops out under load is
the known weak point of forwarding a 230400-baud stream over USB/IP — see
Troubleshooting.

## Step 7 — Feed it into SLAM

Everything downstream already expects `/scan` in `laser_link`, so the real
sensor drops straight in. The one change is sim time:

```bash
# terminal 1
ros2 launch perceptron_robot_bringup lidar_real.launch.py rviz:=false

# terminal 2
ros2 launch perceptron_navigation slam.launch.py use_sim_time:=false
```

> `config/slam_toolbox.yaml` ships with `use_sim_time: true` for Gazebo. The
> launch argument above overrides it, so you do not need to edit the file — but
> if you forget the argument, slam_toolbox will block forever waiting on a
> `/clock` that never comes, with no error message. That silent hang is the most
> common first-run mistake.

Without wheel odometry there is no `odom -> base_footprint`, so slam_toolbox has
nothing to anchor scans against and the map will smear as soon as you move the
sensor. For a bench test that is fine — hold it still and confirm the room walls
appear. For a real map, run the hardware bringup so odometry is publishing too.

---

## Troubleshooting

**`usbipd list` shows nothing plausible.** The kit's USB adapter must be seated
and the lidar's JST cable connected. Check Windows Device Manager for a COM port
under "Ports (COM & LPT)" — if Windows cannot see it, WSL never will.

**`usbipd attach` fails with "device is not shared".** You skipped
`usbipd bind --busid <id>`, which needs an Administrator prompt.

**Device attaches, but no `/dev/ttyUSB*`.** The module is not loaded:
`sudo modprobe cp210x && sudo modprobe ch341`, then `dmesg | tail`.

**`Permission denied` on the port.** You are not in `dialout` yet — that group
change only applies to new sessions. Fully restart WSL from PowerShell with
`wsl --shutdown`. As a one-off, `sudo chmod 666 /dev/ldlidar` works.

**Driver connects but publishes nothing.** Almost always the baud rate or the
wrong product name. The D500 is `LDLiDAR_LD19` at `230400`. If you have the
STL-27L instead, it is `921600`.

**Scan appears mirrored or rotated.** Flip `laser_scan_dir` in
`launch/lidar_real.launch.py`. Get this right on the bench: a mirrored scan
produces a map that looks plausible and is wrong.

**`/scan` rate stutters or drops out.** USB/IP adds latency and is sensitive to
host load, and a continuously streaming 230400-baud device is the worst case for
it. Close Gazebo and other heavy processes; if it persists, this is a limit of
WSL rather than something to tune. For sustained real-robot work, run the driver
on the Jetson and point WSL at it over `ROS_DOMAIN_ID` on the same network —
that removes USB/IP from the path entirely.

**RViz is black or will not open.** WSLg needs a recent Windows 11, which you
have. Try `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch ...` — the same software-rendering
fallback documented for Gazebo in `MANUAL.md`.
