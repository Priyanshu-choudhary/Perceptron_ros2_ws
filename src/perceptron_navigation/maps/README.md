# maps

Saved occupancy grids live here, as a `.pgm` image plus a `.yaml` describing its
resolution and origin. The directory starts empty: you produce a map by driving
the robot around with SLAM running.

```bash
ros2 launch perceptron_navigation nav_simulation.launch.py slam:=true
# drive around until the room is closed up in RViz, then:
ros2 run nav2_map_server map_saver_cli -f <workspace>/src/perceptron_navigation/maps/room_map
colcon build --symlink-install --packages-select perceptron_navigation
```

`navigation.launch.py` defaults to `room_map.yaml` here, so keep that name or
pass `map:=/path/to/other_map.yaml`.

A saved map is tied to the `map` frame origin, which is wherever the robot
happened to be when SLAM started. Spawn the robot at the same pose you mapped
from, or set the initial pose in RViz with **2D Pose Estimate** before
navigating.
