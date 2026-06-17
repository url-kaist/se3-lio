import numpy as np
import pytest

from _bench_import import evaluate as ev


def _rot_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _quat_xyzw_from_rot(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    s = np.sqrt(tr + 1.0) * 2
    w = 0.25 * s
    x = (R[2, 1] - R[1, 2]) / s
    y = (R[0, 2] - R[2, 0]) / s
    z = (R[1, 0] - R[0, 1]) / s
    return np.array([x, y, z, w])


def _write_tum(path, stamps, pos, quat):
    with open(path, "w") as f:
        for t, p, q in zip(stamps, pos, quat):
            f.write(
                f"{t:.9f} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} "
                f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n"
            )


def _traj(n=50):
    t = np.linspace(0.0, 5.0, n)
    pos = np.stack([t, np.sin(t), 0.3 * t], axis=1)
    quat = np.tile(np.array([0.0, 0.0, 0.0, 1.0]), (n, 1))
    return t, pos, quat


def test_umeyama_recovers_known_rt():
    rng = np.random.default_rng(0)
    src = rng.normal(size=(40, 3))
    R_true = _rot_z(0.7)
    t_true = np.array([1.0, -2.0, 0.5])
    dst = (R_true @ src.T).T + t_true
    R, t = ev.umeyama_rigid(src, dst)
    assert np.allclose(R, R_true, atol=1e-9)
    assert np.allclose(t, t_true, atol=1e-9)


def test_ate_zero_after_rigid_alignment(tmp_path):
    stamps, pos, quat = _traj()
    R = _rot_z(0.9)
    t = np.array([3.0, -1.0, 2.0])
    gt_pos = (R @ pos.T).T + t

    est = tmp_path / "est.tum"
    gt = tmp_path / "gt.tum"
    _write_tum(est, stamps, pos, quat)
    _write_tum(gt, stamps, gt_pos, quat)

    res = ev.evaluate(str(est), str(gt))
    assert res["n_assoc"] == len(stamps)
    assert res["ate_rmse"] < 1e-6
    assert res["ate_max"] < 1e-6


def test_ate_constant_offset(tmp_path):
    stamps, pos, quat = _traj()
    # Constant translation offset is removed by alignment, so inject a
    # non-rigid error: shift one axis by a fixed amount only on even samples.
    gt_pos = pos.copy()
    gt_pos[:, 0] += 0.1  # pure translation -> alignment cancels it
    est = tmp_path / "est.tum"
    gt = tmp_path / "gt.tum"
    _write_tum(est, stamps, pos, quat)
    _write_tum(gt, stamps, gt_pos, quat)
    res = ev.evaluate(str(est), str(gt))
    assert res["ate_rmse"] < 1e-6  # rigid alignment absorbs constant offset

    # Now a genuine non-rigid distortion: scale x by 1.1 (alignment can't undo).
    gt_pos2 = pos.copy()
    gt_pos2[:, 0] *= 1.1
    gt2 = tmp_path / "gt2.tum"
    _write_tum(gt2, stamps, gt_pos2, quat)
    res2 = ev.evaluate(str(est), str(gt2))
    assert res2["ate_rmse"] > 1e-3
    assert res2["ate_mean"] <= res2["ate_max"]


def test_too_few_associations_flagged(tmp_path):
    stamps, pos, quat = _traj(n=5)
    est = tmp_path / "est.tum"
    # GT timestamps far away -> no associations within max_dt
    gt = tmp_path / "gt.tum"
    _write_tum(est, stamps, pos, quat)
    _write_tum(gt, stamps + 100.0, pos, quat)
    res = ev.evaluate(str(est), str(gt))
    assert res["n_assoc"] == 0
    assert res["ate_rmse"] == float("inf")
