"""Publishing a fixed grid must not move it with sub-cell camera motion."""
from types import SimpleNamespace
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from elevation_mapping_cupy.elevation_mapping_node import ElevationMappingNode

@pytest.mark.parametrize('camera', [(1.01, -2.04), (1.049, -1.951), (1.051, -1.949)])
def test_publish_uses_grid_center(camera):
    published = []
    node = SimpleNamespace(
        _map_q=object(), _map_t=SimpleNamespace(x=camera[0], y=camera[1]),
        _get_map_center=lambda: np.array([1., -2., .7]),
        map_frame='world', _last_t=Time(sec=10),
        _map=SimpleNamespace(resolution=.1, cell_n=102),
        my_publishers={'map': {'basic_layers': [], 'layers': []}},
        _publishers_dict={'map': SimpleNamespace(publish=published.append)},
    )
    ElevationMappingNode.publish_map(node, 'map')
    pose=published[0].info.pose
    assert pose.position.x == 1.
    assert pose.position.y == -2.
    assert pose.position.z == 0.
    assert pose.orientation.w == 1.


def test_extrapolation_never_substitutes_latest_pose():
    import tf2_ros
    calls = []
    warnings = []
    stamp = Time(sec=123)
    def lookup(target, source, requested):
        calls.append(requested)
        if len(calls) == 1:
            raise tf2_ros.ExtrapolationException('requested time is not available')
        return object()  # A latest-pose fallback would incorrectly succeed.
    node = SimpleNamespace(_tf_buffer=SimpleNamespace(lookup_transform=lookup),
                           get_logger=lambda: SimpleNamespace(warning=lambda text, **kw: warnings.append(text)))
    assert ElevationMappingNode.safe_lookup_transform(node, 'world', 'camera', stamp) is None
    assert calls == [stamp]
    assert warnings
