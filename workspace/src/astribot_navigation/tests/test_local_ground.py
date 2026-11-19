from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from real_navigation.core import validate_config
from real_navigation.geometry import Geometry
from real_navigation.ground import ground_plane, height_above_ground

CONFIG = yaml.safe_load((ROOT / 'scripts/real_navigation/navigation.yaml').read_text())


def test_slope_supported_by_floor_preserves_small_obstacle():
    rng = np.random.default_rng(17)
    xy = rng.uniform(-3., 3., (12000, 2))
    floor = -.11 + .024 * xy[:, 0] - .013 * xy[:, 1]
    raised = (abs(xy[:, 0] - 1.7) < .22) & (abs(xy[:, 1]) < .22)
    points = np.column_stack((xy, floor + .03 * raised + rng.normal(0., .001, len(xy))))
    plane = ground_plane(points, -.11)
    np.testing.assert_allclose(plane, [.024, -.013, -.11], atol=.003)
    heights = height_above_ground(points, -.11)
    assert raised.any() and (heights[raised] > .02).all()
    assert (heights[~raised] < .02).all()


def test_sparse_or_one_sided_obstacle_does_not_become_floor():
    xy = np.linspace(1., 2., 100)
    points = np.column_stack((xy, np.zeros_like(xy), np.full_like(xy, -.08)))
    assert ground_plane(points, -.11) is None
    assert np.min(height_above_ground(points, -.11)) > .02


@pytest.mark.parametrize('rear', [False, True])
def test_ground_relative_cloud_ignores_vertical_odom_drift(rear):
    config = CONFIG
    g = Geometry(config)
    nominal = config['ground_z_in_base']['back' if rear else 'front']
    points = np.array([[2., 0., nominal], [2., .2, nominal + .03], [2., -.2, nominal + .16]])
    lidar = (points - g.base_lidar[:3, 3]) @ np.linalg.inv(g.base_lidar[:3, :3]).T
    original, scan = g.cloud(lidar, np.eye(4), rear)
    assert len(original) == 2 and np.isfinite(scan).sum() == 1
    pose = np.eye(4)
    pose[2, 3] = .25
    shifted, new_scan = g.cloud(lidar, pose, rear)
    np.testing.assert_allclose(shifted, original)
    np.testing.assert_allclose(new_scan, scan)


def test_historical_config_keeps_its_original_height_reference():
    old = {k: v for k, v in CONFIG.items() if k not in ('ground_z_in_base', 'ground_slice_height')}
    g = Geometry(old)
    p = np.array([[2., 0., 0.]])
    _, near = g.cloud(p, np.eye(4))
    pose = np.eye(4)
    pose[2, 3] = .25
    _, far = g.cloud(p, pose)
    assert np.isfinite(near).sum() == 1 and not np.isfinite(far).any()


def test_partial_ground_config_is_rejected():
    with pytest.raises(ValueError):
        validate_config(dict(CONFIG, ground_z_in_base={'front': -.11}))
