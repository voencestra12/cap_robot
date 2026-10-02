import ast
from collections import deque
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


# Load the actual estimator in aruco_calib.py without requiring ROS for math tests.
_path = Path(__file__).resolve().parents[1] / 'cap_robot/aruco_calib.py'
_tree = ast.parse(_path.read_text())
_class = next(node for node in _tree.body
              if isinstance(node, ast.ClassDef) and node.name == 'StaticPoseEstimator')
_module = ast.fix_missing_locations(ast.Module(body=[_class], type_ignores=[]))
_namespace = {'deque': deque, 'np': np, 'R': Rotation}
exec(compile(_module, str(_path), 'exec'), _namespace)
StaticPoseEstimator = _namespace['StaticPoseEstimator']


def test_requires_count_duration_and_distinct_frames():
    estimator = StaticPoseEstimator(min_samples=3)
    for stamp in [0, 0, 0, 100_000_000, 200_000_000]:
        assert estimator.add(stamp, [0, 0, 1], [0, 0, 0]) is None
    assert estimator.add(1_000_000_000, [0, 0, 1], [0, 0, 0]) is not None


def test_rotation_wraparound_uses_so3_mean():
    estimator = StaticPoseEstimator(min_samples=4)
    for i, angle in enumerate([179.5, -179.5, 179.5, -179.5]):
        result = estimator.add(i * 500_000_000, [0, 0, 1],
                               Rotation.from_euler('z', angle, degrees=True).as_rotvec())
    translation, quaternion = result
    np.testing.assert_allclose(translation, [0, 0, 1])
    relative = Rotation.from_quat(quaternion).inv() * Rotation.from_euler('z', 180, degrees=True)
    assert relative.magnitude() < 1e-8


@pytest.mark.parametrize('translation,rotvec', [([0.2, 0, 1], [0, 0, 0]),
                                               ([0, 0, 1], [0, 0, 0.5])])
def test_bimodal_observations_do_not_lock(translation, rotvec):
    estimator = StaticPoseEstimator(min_samples=10)
    for i in range(60):
        assert estimator.add(i * 100_000_000,
                             translation if i % 2 else [0, 0, 1],
                             rotvec if i % 2 else [0, 0, 0]) is None


def test_outliers_rejected_and_early_bad_samples_expire():
    estimator = StaticPoseEstimator(min_samples=10)
    for i in range(3):
        assert estimator.add(i * 100_000_000, [i, 0, 2], [0, 0, 1]) is None
    for i in range(10):
        result = estimator.add(6_000_000_000 + i * 200_000_000, [0, 0, 1], [0, 0, 0])
    np.testing.assert_allclose(result[0], [0, 0, 1])


def test_small_outlier_fraction_is_excluded_from_mean():
    estimator = StaticPoseEstimator(min_samples=10)
    assert estimator.add(0, [10, 0, 1], [0, 0, 2]) is None
    for i in range(10):
        result = estimator.add((i + 1) * 200_000_000, [0, 0, 1], [0, 0, 0])
    np.testing.assert_allclose(result[0], [0, 0, 1])
    assert Rotation.from_quat(result[1]).magnitude() < 1e-8


def test_invalid_and_sparse_observations_do_not_lock():
    estimator = StaticPoseEstimator(min_samples=3)
    assert estimator.add(0, [np.nan, 0, 1], [0, 0, 0]) is None
    assert estimator.add(0, [0, 0, 1], [np.inf, 0, 0]) is None
    for i in range(10):
        assert estimator.add(i * 6_000_000_000, [0, 0, 1], [0, 0, 0]) is None


@pytest.mark.parametrize('kwargs', [dict(min_samples=1), dict(min_duration_sec=0),
                                    dict(max_age_sec=0.5), dict(rotation_tolerance_deg=np.nan)])
def test_invalid_limits_rejected(kwargs):
    with pytest.raises(ValueError):
        StaticPoseEstimator(**kwargs)
