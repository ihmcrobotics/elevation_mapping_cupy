"""Exercise real ROS subscriptions, TF, GPU fusion and GridMap publication."""
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from grid_map_msgs.msg import GridMap
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointField, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster

from elevation_mapping_cupy.elevation_mapping_node import ElevationMappingNode


def test_cloud_and_image_reach_published_gridmap(tmp_path):
    config = yaml.safe_load((Path(__file__).parent / 'config/test_integration.yaml').read_text())
    params = config['/elevation_mapping_node']['ros__parameters']
    params['subscribers'] = {
        'cloud': {'topic_name': '/semantic_test/points', 'data_type': 'pointcloud', 'channels': ['person']},
        'camera': {'topic_name': '/semantic_test/image', 'data_type': 'image',
                   'camera_info_topic_name': '/semantic_test/info', 'channels': ['road']},
    }
    params['publishers']['elevation_map']['layers'] = ['elevation', 'person', 'road']
    path = tmp_path / 'params.yaml'
    path.write_text(yaml.safe_dump(config))
    rclpy.init(args=['--ros-args', '--params-file', str(path)])
    mapper = None
    fixture = None
    executor = SingleThreadedExecutor()
    try:
        mapper = ElevationMappingNode()
        fixture = Node('semantic_test_fixture')
        executor.add_node(mapper)
        executor.add_node(fixture)
        broadcaster = StaticTransformBroadcaster(fixture)
        transforms = []
        for child in ['base_link', 'camera_optical']:
            tf = TransformStamped()
            tf.header.frame_id = 'map'
            tf.child_frame_id = child
            tf.header.stamp = fixture.get_clock().now().to_msg()
            if child == 'camera_optical':
                tf.transform.translation.z = 2.0
                tf.transform.rotation.x = 1.0
                tf.transform.rotation.w = 0.0
            else:
                tf.transform.rotation.w = 1.0
            transforms.append(tf)
        broadcaster.sendTransform(transforms)
        cloud_pub = fixture.create_publisher(PointCloud2, '/semantic_test/points', qos_profile_sensor_data)
        image_pub = fixture.create_publisher(Image, '/semantic_test/image', qos_profile_sensor_data)
        info_pub = fixture.create_publisher(CameraInfo, '/semantic_test/info', qos_profile_sensor_data)
        received = []
        fixture.create_subscription(GridMap, '/elevation_mapping_node/elevation_map', received.append, 10)
        fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(['x', 'y', 'z', 'person'])]
        points = np.array([[x, y, 0, 0.8] for x in np.arange(-0.8, 0.9, 0.1)
                           for y in np.arange(-0.8, 0.9, 0.1)], dtype=np.float32)
        bridge = CvBridge()
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            stamp = fixture.get_clock().now().to_msg()
            cloud_pub.publish(point_cloud2.create_cloud(Header(stamp=stamp, frame_id='map'), fields, points))
            msg = bridge.cv2_to_imgmsg(np.full((80, 80), 0.6, np.float32), encoding='32FC1')
            msg.header = Header(stamp=stamp, frame_id='camera_optical')
            info = CameraInfo(header=msg.header, height=80, width=80, distortion_model='plumb_bob',
                              d=[0.0]*5, k=[20.0, 0.0, 40.0, 0.0, 20.0, 40.0, 0.0, 0.0, 1.0])
            image_pub.publish(msg)
            info_pub.publish(info)
            for _ in range(10):
                executor.spin_once(timeout_sec=0.02)
            for grid in received[-5:]:
                layers = dict(zip(grid.layers, grid.data))
                if all(name in layers and np.nanmax(layers[name].data) > 0.1 for name in ['person', 'road']):
                    assert mapper._image_process_counter > 0
                    assert mapper._pointcloud_process_counter > 0
                    return
        raise AssertionError(f'No fused GridMap: received={len(received)}, '
                             f'clouds={mapper._pointcloud_process_counter}, images={mapper._image_process_counter}, '
                             f'layers={mapper._map.semantic_map.layer_names}, '
                             f'maxima={mapper._map.semantic_map.semantic_map.max(axis=(1, 2))}, '
                             f'valid={mapper._map.elevation_map[2].max()}, heights={mapper._map.elevation_map[0].min(), mapper._map.elevation_map[0].max()}, correspondences={mapper._map.valid_correspondence.sum()}, '
                             f'last_layers={received[-1].layers if received else []}')
    finally:
        executor.shutdown()
        if mapper is not None:
            mapper.destroy_node()
        if fixture is not None:
            fixture.destroy_node()
        rclpy.try_shutdown()
