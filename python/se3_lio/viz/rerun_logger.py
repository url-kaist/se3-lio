"""Rerun logging for SE(3)-LIO.

Per frame we transform the LiDAR-frame scan into the world frame with the
estimated pose and log it under a unique entity path. Rerun accumulates these
across the timeline, so playing it back shows the map being built. The
trajectory line and current sensor pose are logged alongside.
"""

import numpy as np


def lidar_to_world(pose, extrinsic, pts):
    """Transform LiDAR-frame points (N, 3) into the world frame.

    Mirrors the binding's convention: extrinsic maps LiDAR->body, pose maps
    body->world, so world = pose @ extrinsic @ [pt; 1].
    """
    pose = np.asarray(pose, dtype=float)
    extrinsic = np.asarray(extrinsic, dtype=float)
    pts = np.asarray(pts, dtype=float)
    T = pose @ extrinsic
    return pts @ T[:3, :3].T + T[:3, 3]


def _set_time(rr, timeline, seconds):
    """Set the active time, across rerun API versions (<=0.22 vs >=0.23)."""
    if hasattr(rr, "set_time_seconds"):
        rr.set_time_seconds(timeline, seconds)
    else:
        rr.set_time(timeline, timestamp=seconds)


def _height_to_rgb(z, zmin, zmax):
    """Blue -> green -> red gradient over [zmin, zmax]; returns (N, 3) uint8."""
    span = zmax - zmin
    t = np.zeros_like(z) if span <= 1e-6 else np.clip((z - zmin) / span, 0.0, 1.0)
    r = np.clip(2.0 * t - 1.0, 0.0, 1.0)
    b = np.clip(1.0 - 2.0 * t, 0.0, 1.0)
    g = 1.0 - r - b
    return (np.stack([r, g, b], axis=1) * 255).astype(np.uint8)


class RerunLogger:
    """Logs trajectory + per-frame world scan so Rerun accumulates them into a map."""

    def __init__(self, extrinsic, app_id="se3_lio"):
        import rerun as rr

        self._rr = rr
        self._extrinsic = np.asarray(extrinsic, dtype=float)
        self._traj = []
        self._zmin = np.inf
        self._zmax = -np.inf
        self._i = 0
        rr.init(app_id)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    def log_frame(self, stamp, pose, scan_pts):
        rr = self._rr
        pose = np.asarray(pose, dtype=float)
        world = lidar_to_world(pose, self._extrinsic, scan_pts)

        _set_time(rr, "stamp", float(stamp))

        self._zmin = min(self._zmin, float(world[:, 2].min()))
        self._zmax = max(self._zmax, float(world[:, 2].max()))
        colors = _height_to_rgb(world[:, 2], self._zmin, self._zmax)
        rr.log(f"world/map/{self._i:05d}", rr.Points3D(world, colors=colors, radii=0.03))
        self._i += 1

        pos = pose[:3, 3]
        self._traj.append(pos)
        rr.log("world/trajectory", rr.LineStrips3D([np.array(self._traj)], radii=0.05))
        rr.log("world/sensor", rr.Transform3D(translation=pos, mat3x3=pose[:3, :3]))

    def save(self, path):
        self._rr.save(path)

    def spawn(self):
        self._rr.spawn()


def read_rrd_trajectory(path):
    """Read back the logged sensor trajectory from a .rrd recording.

    Returns (stamps (M,), positions (M, 3)). Used by the G2 smoke test and the
    G3 check to compare the recording against the estimated trajectory.
    """
    import pyarrow as pa
    import rerun as rr

    recording = rr.dataframe.load_recording(path)
    view = recording.view(index="stamp", contents={"world/sensor": ["Translation3D"]})
    table = view.select().read_all()

    stamp_col = table.column("stamp").combine_chunks()
    if pa.types.is_timestamp(stamp_col.type):
        stamps = stamp_col.cast(pa.int64()).to_numpy() / 1e9
    else:
        stamps = np.asarray(stamp_col.to_numpy(zero_copy_only=False), dtype=float)

    col = table.column("/world/sensor:Translation3D").to_pylist()
    positions = np.array([np.asarray(v, dtype=float).reshape(-1)[:3] for v in col])
    return stamps, positions
