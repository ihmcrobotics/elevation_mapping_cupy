# Elevation Mapping CuPy (ROS 2, Jazzy)

This repo contains:
- `elevation_map_msgs`: message definitions (ament_cmake).
- `elevation_mapping_cupy`: GPU-accelerated elevation mapping node (Python, CuPy).

## IHMC Jazzy branch

`ihmc-jazzy` extends the preserved upstream `release/jazzy` baseline with
semantic point-cloud fields and calibrated RGB/semantic image fusion.
See [IHMC_JAZZY.md](IHMC_JAZZY.md) for the Docker build, configuration,
provenance and GPU/ROS validation commands.

The core synthetic PointCloud2 demonstration below remains available.

## Requirements

- ROS 2 Jazzy (Ubuntu 24.04)
- NVIDIA GPU + CUDA (CuPy uses CUDA)
- Python deps not managed by rosdep:
  - `cupy-cuda12x`
  - `torch` (CUDA build)
  - `simple-parsing`

## Quick Start (Docker, Recommended)

```bash
cd ~/ros2_ws/src/elevation_mapping_cupy
docker build -f docker/Dockerfile.x64 -t elevation_mapping_cupy:jazzy .

# Build + test in a mounted Jazzy workspace.
docker run --rm --gpus all --net=host \
  -v ~/ros2_ws:/ws -w /ws elevation_mapping_cupy:jazzy bash -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    colcon build --symlink-install \
      --packages-select elevation_map_msgs elevation_mapping_cupy
    source install/setup.bash
    colcon test --packages-select elevation_mapping_cupy \
      --event-handlers console_direct+
    colcon test-result --verbose
  '
```

## Run (golden path)

```bash
# Headless:
docker run --rm --gpus all --net=host \
  -v ~/ros2_ws:/ws -w /ws elevation_mapping_cupy:jazzy bash -lc '
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 launch elevation_mapping_cupy synthetic_depth_demo.launch.py launch_rviz:=false
  '
```

What you should see:
- TF tree contains `map -> base_link`
- RViz shows the GridMap updating on `/elevation_mapping_node/elevation_map`
- As the robot frame moves, the map stays consistent in the world (no axis swap)

Tip: if you run `ros2 topic echo` / tooling from a *second* container, set `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`
to avoid shared-memory transport issues between containers.

## Build (native, non-Docker)

Native install is possible, but you must install the Python CUDA deps yourself (see above).

## Tests (regressions)

```bash
cd ~/ros2_ws
colcon test --packages-select elevation_mapping_cupy --event-handlers console_direct+
colcon test-result --verbose
```

The suite includes:
- Pure-python config sanity checks (no ROS1 substitutions, no deprecated keys).
- GPU smoke tests (CuPy + kernel compile + one update step).
- launch_testing integration:
  - TF -> map shifting -> GridMap publishing
  - save/load services (rosbag2)

## Not Supported / Disabled

- `plane_segmentation/`: disabled by default via `COLCON_IGNORE` (heavy deps, not part of bring-up).
