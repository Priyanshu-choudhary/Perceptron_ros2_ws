#!/usr/bin/env bash
# One-time WSL-side setup for the physical LDROBOT D500 (STL-19P) lidar.
#
# Run this INSIDE WSL, once, after usbipd-win is installed on Windows:
#
#     bash tools/setup_d500_wsl.sh
#
# It does four things:
#   1. loads the USB-serial kernel modules WSL2 ships but does not autoload
#   2. installs a udev rule so the lidar shows up as /dev/ldlidar
#   3. clones the LDROBOT driver into src/ if it is not already there
#   4. tells you what to do next
#
# It deliberately does NOT build; run colcon yourself so you see the errors.

set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DRIVER_DIR="$WS/src/ldlidar_stl_ros2"

echo "==> workspace: $WS"

# --- 1. kernel modules -------------------------------------------------------
# The WSL2 kernel has cp210x / ch341 / cdc-acm built as modules but nothing
# loads them until a matching device appears, and on WSL the hotplug path is
# unreliable. Loading them up front is harmless and saves a confusing "device
# attached but no /dev/ttyUSB0".
echo "==> loading USB serial modules"
for m in usbserial cp210x ch341 ftdi_sio cdc-acm; do
  sudo modprobe "$m" 2>/dev/null && echo "    loaded $m" || echo "    skipped $m"
done

# Make them load on every WSL boot.
if [ ! -f /etc/modules-load.d/usb-serial.conf ]; then
  echo "==> persisting module load across WSL restarts"
  printf 'usbserial\ncp210x\nch341\nftdi_sio\ncdc-acm\n' \
    | sudo tee /etc/modules-load.d/usb-serial.conf >/dev/null
fi

# --- 2. udev rule ------------------------------------------------------------
# Two vendor IDs because the D500 kit ships with either a CP2102 (10c4:ea60) or
# a CH340 (1a86:7523) USB-to-serial adapter depending on batch. The rule gives
# the port to the dialout group and creates a stable /dev/ldlidar symlink, so
# the launch file does not care whether it enumerated as ttyUSB0 or ttyUSB1.
echo "==> installing udev rule -> /dev/ldlidar"
sudo tee /etc/udev/rules.d/99-ldlidar.rules >/dev/null <<'RULES'
# LDROBOT D500 / STL-19P via CP2102
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ldlidar"
# LDROBOT D500 / STL-19P via CH340
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout", SYMLINK+="ldlidar"
RULES

sudo service udev restart >/dev/null 2>&1 || true
sudo udevadm control --reload-rules 2>/dev/null || true
sudo udevadm trigger 2>/dev/null || true

sudo usermod -aG dialout "$USER" || true

# --- 3. driver source --------------------------------------------------------
if [ -d "$DRIVER_DIR" ]; then
  echo "==> driver already present at src/ldlidar_stl_ros2"
else
  echo "==> cloning LDROBOT driver"
  git clone --depth 1 https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git "$DRIVER_DIR"
fi

# --- 4. next steps -----------------------------------------------------------
cat <<'NEXT'

==> setup done. Next:

  On Windows (PowerShell as Administrator), attach the lidar to WSL:
      usbipd list
      usbipd bind   --busid <BUSID>
      usbipd attach --wsl --busid <BUSID>

  Back in WSL, confirm the device appeared:
      ls -l /dev/ldlidar

  Then build and run:
      colcon build --symlink-install --packages-select ldlidar_stl_ros2 perceptron_robot_bringup
      source install/setup.bash
      ros2 launch perceptron_robot_bringup lidar_real.launch.py

NEXT
