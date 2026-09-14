# Vendored packages

Two packages under `src/` are upstream clones, not authored here. They are
git-ignored to keep this repository small (the AWS world alone is ~183 MB of
meshes and textures), so a fresh clone of this workspace will **not** build
until they are restored.

## Restore after a fresh clone

```bash
cd src

git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-small-house-world.git \
    aws_robomaker_small_house_world

git clone -b master https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git \
    ldlidar_stl_ros2
```

## Versions in use

| package | remote | branch | commit |
| --- | --- | --- | --- |
| `aws_robomaker_small_house_world` | `aws-robotics/aws-robomaker-small-house-world` | `ros2` | `ff9631c` |
| `ldlidar_stl_ros2` | `ldrobotSensorTeam/ldlidar_stl_ros2` | `master` | `bf668a8` |

If a vendored package ever needs local patches, stop ignoring it and either
commit it outright or convert it to a proper git submodule — a patched clone
that git cannot see is the one state guaranteed to be lost.
