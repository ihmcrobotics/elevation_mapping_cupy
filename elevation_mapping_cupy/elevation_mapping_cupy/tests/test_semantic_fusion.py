"""GPU regressions for the IHMC Jazzy semantic/image backport."""
from pathlib import Path

import cupy as cp
import numpy as np
import pytest

from elevation_mapping_cupy import ElevationMap, Parameter


@pytest.fixture
def emap():
    root = Path(__file__).resolve().parents[2]
    p = Parameter(
        use_chainer=False,
        weight_file=str(root / 'config/core/weights.dat'),
        plugin_config_file=str(root / 'config/core/plugin_config.yaml'),
    )
    p.resolution = 0.2
    p.map_length = 4.0
    p.enable_visibility_cleanup = False
    p.enable_overlap_clearance = False
    p.enable_drift_compensation = False
    p.min_valid_distance = 0.0
    p.subscriber_cfg = {'camera': {'data_type': 'image'}}
    p.update()
    return ElevationMap(p)


def test_semantic_cloud_fuses_publishes_shifts_and_clears(emap):
    points = np.array([[0.4, 0.2, 0.0, 0.8], [0.8, -0.4, 0.0, 0.6]], np.float32)
    emap.input_pointcloud(points, ['x', 'y', 'z', 'person'], np.eye(3), np.zeros(3), 0, 0)
    assert emap.exists_layer('person')
    layer = emap.semantic_map.semantic_map[0].get()
    assert np.count_nonzero(layer) == 2
    assert np.max(layer) > 0.0
    # Semantic cells must align with valid geometry, including asymmetric X/Y.
    assert np.all(emap.elevation_map[2].get()[layer > 0] > 0)
    published = np.zeros((emap.cell_n - 2, emap.cell_n - 2), np.float32)
    emap.get_map_with_name_ref('person', published)
    assert np.count_nonzero(published) == 2
    emap.shift_map_xy(cp.array([1, 0]))
    np.testing.assert_array_equal(emap.semantic_map.semantic_map[0].get(), np.roll(layer, 1, axis=1))
    emap.clear()
    assert not bool(cp.any(emap.semantic_map.semantic_map))


@pytest.mark.parametrize("camera_offset", [0.0, 0.037])
def test_image_fuses_scores_and_rgb_with_correct_xy_projection(emap, camera_offset):
    emap.elevation_map[2, 1:-1, 1:-1] = 1
    rotation = np.diag([1, -1, -1]).astype(np.float32)
    translation = np.array([camera_offset, 0, 2], np.float32)
    k = np.array([[20, 0, 40], [0, 20, 40], [0, 0, 1]], np.float32)
    image = np.tile(np.arange(80, dtype=np.float32) / 80, (80, 1))
    emap.input_image([image], ['person'], rotation, translation, k, np.zeros(5), 'plumb_bob', 80, 80)
    layer = emap.semantic_map.semantic_map[0].get()
    center = emap.cell_n // 2
    assert layer[center, center + 2] > layer[center, center - 2] > 0
    assert layer[center + 2, center] == pytest.approx(layer[center - 2, center])
    rgb = [np.full((80, 80), c, np.float32) for c in (255, 64, 16)]
    emap.input_image(rgb, ['rgb'], rotation, translation, k, np.zeros((5, 1)), 'plumb_bob', 80, 80)
    packed = emap.semantic_map.semantic_map[1].get().view(np.uint32)
    assert packed[center, center] == (255 << 16) + (64 << 8) + 16
