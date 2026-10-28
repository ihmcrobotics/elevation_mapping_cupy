# IHMC Jazzy semantic/image fusion

All IHMC changes live on `ihmc-jazzy`, based on upstream `release/jazzy`
commit `68e18856567e5dc406418e7c816ef0e4c00d4987`. The original `main` and
`release/jazzy` branches are preserved.

The ROS 2 subscribers, semantic_sensor package and example launches are
backported from upstream commit `2c9b25ab12d960cc9743031bee0224224997c961`,
with example configuration corrections from `a77b2f3`. The semantic map,
fusion plugins and image kernels come from the original MIT-licensed
implementation at `20a8a26b67a995b43eb44c23568854d1fed82a52`.
IHMC restores the missing backend connections and corrects Jazzy map axes,
camera extrinsics, CUDA argument types and sensor QoS.

## Build and run

From this repository root, using the existing Hydra Jazzy image:

```bash
./docker/build_ihmc.sh
./docker/run_ihmc.sh
```

This builds `ihmc-elevation-mapping:jazzy` as a separate image. It installs
CuPy 13.6.0, simple-parsing, GridMap and ROS dependencies and builds
`elevation_map_msgs`, `ros2_numpy`, `semantic_sensor` and `elevation_mapping_cupy`.
Torch/torchvision and CUDA 12.8 are inherited from `ihmc-reasoning-hydra:jazzy`.
The ros2_numpy dependency is pinned to its Jazzy commit in the Dockerfile.
The default command runs the headless synthetic geometry demo on DDS domain 173.

For your own robot configuration (directory must be an absolute host path):

```bash
ELEVATION_CONFIG_DIR=/absolute/path/to/config ROS_DOMAIN_ID=42 \
  ./docker/run_ihmc.sh ros2 run elevation_mapping_cupy elevation_mapping_node.py \
  --ros-args \
  --params-file /opt/elevation_ws/install/elevation_mapping_cupy/share/elevation_mapping_cupy/config/core/core_param.yaml \
  --params-file /config/robot.yaml
```

The robot supplies PointCloud2, Image/CameraInfo and timestamped TF. Depth
must first be converted to PointCloud2. Image fusion projects onto valid
height cells; images alone cannot create elevation geometry.

## Semantic configuration

Example robot.yaml (replace topics and frames to match the robot):

```yaml
/elevation_mapping_node:
  ros__parameters:
    map_frame: map
    base_frame: base_link
    subscribers:
      geometry:
        topic_name: /camera/depth/points
        data_type: pointcloud
      semantic:
        topic_name: /camera/semantic_scores
        camera_info_topic_name: /camera/camera_info
        channels: [person, floor]
        data_type: image
    image_channel_fusions:
      default: exponential
    publishers:
      elevation_map:
        layers: [elevation, traversability, person, floor]
        basic_layers: [elevation]
        fps: 5.0
```

Semantic images contain one float score channel per configured class, in
exactly the same order. Discrete class IDs must be converted into class-score
channels first. Alternatively, publish ChannelInfo and set
`channel_info_topic_name` instead of static `channels`.

For RGB fusion use an RGB-ordered three-channel image, `channels: [rgb]`,
`image_channel_fusions: {rgb: color}` and add `rgb` to the publisher layers.
Rectify fisheye/equidistant images before fusion. CameraInfo must describe
the delivered image size and optical frame.

For semantic point clouds, add scalar FLOAT32 fields (e.g. person/floor),
list them in the pointcloud subscriber's `channels`, and set
`pointcloud_channel_fusions: {default: class_average}`. RGB uses a packed
FLOAT32 `rgb` field and the `color` fusion rule.

The restored `semantic_sensor` can produce scores using torchvision models;
first use may download pretrained weights. Existing Hydra/YOLO outputs are
not automatically subscribed: they need the matching score-image or cloud
message contract above. The TurtleBot semantic examples additionally require
simulation packages and are not installed by the IHMC runtime image.

## Validation

```bash
./docker/run_ihmc.sh bash -c '
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -p no:cacheprovider \
    src/elevation_mapping_cupy/elevation_mapping_cupy/elevation_mapping_cupy/tests/test_semantic_fusion.py \
    src/elevation_mapping_cupy/elevation_mapping_cupy/test/test_semantic_ros_pipeline.py \
    src/elevation_mapping_cupy/sensor_processing/semantic_sensor/test -q
'
```

The GPU tests check semantic cloud fusion, layer publication, map shifts,
clear, image projection and packed RGB. The ROS test publishes real
best-effort sensor messages and TF and requires both image and cloud score
layers to appear in a GridMap.

Validated on the local RTX 4070 GPU with the IHMC Jazzy image: colcon reports
50 tests, 0 errors, 0 failures, 0 skipped; the semantic_sensor configuration
and launch suite reports 3 passed. Both semantic node imports and CUDA
availability in CuPy/PyTorch were checked. Live robot/YOLO inference and the
TurtleBot simulation were not exercised.

The old, unregistered `tests/test_elevation_mapping.py` still references
`../../../config/weights.dat`; broad pytest collection hits 72 fixture errors
there. Use the maintained colcon suite and the explicit semantic tests above.
