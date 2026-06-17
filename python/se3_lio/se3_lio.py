"""High-level Python API over the raw pybind module."""

import numpy as np

from se3_lio.pybind import se3_lio_pybind as _pb
from se3_lio.config import SE3LIOConfig


class SE3LIO:
    """SE(3) LiDAR-Inertial Odometry.

    Wraps the C++ core. `register_frame` reproduces the ROS2 node's per-frame
    data path (build measurement -> apply LiDAR extrinsic -> sort points by
    relative timestamp -> estimatePose) and returns the updated state.
    """

    def __init__(self, config: SE3LIOConfig = None, lidar_extrinsic=None):
        self.config = config if config is not None else SE3LIOConfig()
        self.extrinsic = np.eye(4) if lidar_extrinsic is None else np.asarray(
            lidar_extrinsic, dtype=float
        )
        self._odom = _pb._SE3LIO(self.config.to_pybind(), self.extrinsic)

    def register_frame(self, points, point_times, imu, frame_stamp):
        """Run one odometry step.

        points:      (N, 3) float xyz in the LiDAR frame
        point_times: (N,)   float per-point time offset from frame start [s]
        imu:         (M, 7) float rows of [t, ax, ay, az, gx, gy, gz]
        frame_stamp: float, absolute start time of the scan [s]
        Returns the pybind _State (pose, vel, bg, ba, grav, covariance, ...).
        """
        return self._odom._register_frame(
            np.ascontiguousarray(points, dtype=float),
            np.ascontiguousarray(point_times, dtype=float).ravel(),
            np.ascontiguousarray(imu, dtype=float),
            float(frame_stamp),
        )

    @property
    def last_pose(self):
        return self._odom._last_pose()
