"""Rerun visualization harness — gates G1 and G2.

These run without the C++ binding or a rosbag: G1 checks the LiDAR->world
transform, G2 logs synthetic frames to a .rrd and reads the trajectory back.
The rerun_logger module is loaded by path so importing it does not pull in
se3_lio/__init__ (which imports the compiled binding).
"""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

_MOD_PATH = Path(__file__).resolve().parents[1] / "se3_lio" / "viz" / "rerun_logger.py"
_spec = importlib.util.spec_from_file_location("se3_lio_rerun_logger", _MOD_PATH)
rl = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(rl)


def test_g1_translation_only():
    pose = np.eye(4)
    pose[:3, 3] = [5.0, 0.0, 0.0]
    pts = np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0]])
    world = rl.lidar_to_world(pose, np.eye(4), pts)
    assert np.allclose(world, [[6.0, 0.0, 0.0], [5.0, 2.0, 0.0]])


def test_g1_extrinsic_yaw_90deg():
    # extrinsic rotates LiDAR +x -> world +y; pose then shifts +5 in x.
    ext = np.eye(4)
    ext[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    pose = np.eye(4)
    pose[:3, 3] = [5.0, 0.0, 0.0]
    world = rl.lidar_to_world(pose, ext, np.array([[1.0, 0.0, 0.0]]))
    assert np.allclose(world, [[5.0, 1.0, 0.0]])


def test_g2_rrd_roundtrip(tmp_path):
    pytest.importorskip("rerun")
    n = 5
    logger = rl.RerunLogger(np.eye(4), app_id="se3_lio_test")
    expected = []
    for i in range(n):
        pose = np.eye(4)
        pose[:3, 3] = [float(i), 0.0, 0.0]  # straight line along +x
        expected.append(pose[:3, 3].copy())
        scan = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]])
        logger.log_frame(stamp=float(i), pose=pose, scan_pts=scan)

    out = tmp_path / "smoke.rrd"
    logger.save(str(out))
    assert out.exists() and out.stat().st_size > 0
    assert logger._i == n  # one map entity per frame
    assert len(logger._traj) == n

    stamps, positions = rl.read_rrd_trajectory(str(out))
    assert positions.shape == (n, 3)
    assert np.allclose(stamps, np.arange(n))
    assert np.allclose(positions, np.array(expected))
