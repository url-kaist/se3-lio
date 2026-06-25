"""Polyscope viz unit tests — pure helpers, no GUI or binding needed.

The module is loaded by path so importing it does not pull in se3_lio/__init__
(the compiled binding), and polyscope itself is imported lazily only in the
visualizer's constructor, so these run on a bare host.
"""

import importlib.util
from pathlib import Path

import numpy as np

_MOD = Path(__file__).resolve().parents[1] / "se3_lio" / "viz" / "polyscope_viz.py"
_spec = importlib.util.spec_from_file_location("se3_lio_polyscope_viz", _MOD)
pv = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pv)


def test_lidar_to_world_translation():
    pose = np.eye(4)
    pose[:3, 3] = [5.0, 0.0, 0.0]
    pts = np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0]])
    world = pv._lidar_to_world(pose, np.eye(4), pts)
    assert np.allclose(world, [[6.0, 0.0, 0.0], [5.0, 2.0, 0.0]])


def test_gravity_align_upright_is_identity():
    assert np.allclose(pv._gravity_align([0.0, 0.0, -9.81]), np.eye(3))


def test_gravity_align_flipped_maps_gravity_to_minus_z():
    R = pv._gravity_align([0.0, 0.0, 9.81])
    assert np.allclose(R @ np.array([0.0, 0.0, 1.0]), [0.0, 0.0, -1.0])
