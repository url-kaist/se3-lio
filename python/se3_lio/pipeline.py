"""Offline odometry pipeline — runs SE3LIO over a dataset and collects a trajectory."""

import numpy as np

from se3_lio.se3_lio import SE3LIO


def _rot_to_quat_xyzw(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return np.array([x, y, z, w])


class OdometryPipeline:
    """Run SE3LIO over an iterable dataset of frames and collect the trajectory."""

    def __init__(self, dataset, config, extrinsic=None):
        self.dataset = dataset
        self.odometry = SE3LIO(config, extrinsic)
        self.stamps = []
        self.poses = []  # list of 4x4

    def run(self, progress=True, logger=None):
        frames = self.dataset
        if progress:
            try:
                from tqdm import tqdm

                total = len(self.dataset) if hasattr(self.dataset, "__len__") else None
                frames = tqdm(self.dataset, total=total, desc="SE3-LIO", unit="frame")
            except ImportError:
                pass
        for frame in frames:
            state, _ = self.odometry.register_frame(
                frame.points, frame.point_times, frame.imu, frame.stamp
            )
            self.stamps.append(state.stamp)
            self.poses.append(np.array(state.pose))
            if logger is not None:
                logger.log_frame(state.stamp, self.poses[-1], frame.points, state.grav)
        return self

    def save_tum(self, path):
        with open(path, "w") as f:
            for t, T in zip(self.stamps, self.poses):
                p = T[:3, 3]
                q = _rot_to_quat_xyzw(T[:3, :3])
                f.write(
                    f"{t:.9f} {p[0]:.9f} {p[1]:.9f} {p[2]:.9f} "
                    f"{q[0]:.9f} {q[1]:.9f} {q[2]:.9f} {q[3]:.9f}\n"
                )

    def path_length(self):
        if len(self.poses) < 2:
            return 0.0
        pos = np.array([T[:3, 3] for T in self.poses])
        return float(np.linalg.norm(np.diff(pos, axis=0), axis=1).sum())

    def summary(self):
        n = len(self.poses)
        if n == 0:
            return "no frames processed"
        final = self.poses[-1][:3, 3]
        return (
            f"frames: {n}\n"
            f"path length: {self.path_length():.3f} m\n"
            f"final position: [{final[0]:.3f}, {final[1]:.3f}, {final[2]:.3f}]"
        )
