"""Rerun logging for SE(3)-LIO.

At each keyframe the LiDAR scan is transformed into the world frame and logged
under a unique entity path, so Rerun accumulates the scans into a map across the
timeline; the trajectory line and sensor pose are logged every frame. The whole
world is rotated so gravity points down (z up). Point color/size are not logged
per-point; a blueprint bakes their view defaults (and a dark background) into the
recording, and they stay overridable in the Rerun UI.
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


def _vec_align(a, b):
    """Rotation matrix R (3x3) with R @ a == b, for unit vectors a and b."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if c > 1.0 - 1e-8:
        return np.eye(3)
    if c < -1.0 + 1e-8:
        # Antiparallel (e.g. upside-down IMU): 180 deg about any axis ⊥ to a.
        axis = np.cross(a, [1.0, 0.0, 0.0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, [0.0, 1.0, 0.0])
        axis = axis / np.linalg.norm(axis)
        return 2.0 * np.outer(axis, axis) - np.eye(3)
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def _gravity_align(grav):
    """Rotation that maps the world frame so gravity points along -z (z up)."""
    if grav is None:
        return np.eye(3)
    grav = np.asarray(grav, dtype=float)
    nrm = float(np.linalg.norm(grav))
    if nrm < 1e-9:
        return np.eye(3)
    return _vec_align(grav / nrm, np.array([0.0, 0.0, -1.0]))


class RerunLogger:
    """Logs trajectory + per-frame world scan so Rerun accumulates them into a map."""

    def __init__(self, extrinsic, app_id="se3_lio", keyframe_dist=0.0, save_path=None):
        import os

        import rerun as rr

        self._rr = rr
        self._extrinsic = np.asarray(extrinsic, dtype=float)
        self._keyframe_dist = float(keyframe_dist)
        self._last_kf_pos = None
        self._traj = []
        self._R = None  # gravity-alignment rotation, fixed from the first frame
        self._i = 0
        rr.init(app_id)
        # With a save_path, attach the file sink up front so each log streams to
        # disk instead of accumulating the whole recording in memory.
        if save_path is not None:
            os.makedirs(os.path.dirname(save_path) or ".", exist_ok=True)
            rr.save(save_path)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
        self._send_blueprint()

    def _send_blueprint(self):
        """Bake the default look (dark background, point color/size) into the
        recording so it opens the same way without manual view-default tweaks.
        These are defaults, so they remain overridable in the Rerun UI."""
        import rerun.blueprint as rrb

        rr = self._rr
        rr.send_blueprint(
            rrb.Blueprint(
                rrb.Spatial3DView(
                    origin="/world",
                    background=[18, 18, 26],
                    defaults=[
                        rr.components.Color([200, 200, 200]),
                        rr.components.Radius(0.03),
                    ],
                )
            )
        )

    def log_frame(self, stamp, pose, scan_pts, grav=None):
        rr = self._rr
        pose = np.asarray(pose, dtype=float)
        # Fix the gravity-alignment rotation from the first frame, then apply it
        # to every pose and scan so the streamed world has gravity along -z.
        if self._R is None:
            self._R = _gravity_align(grav)
        R = self._R
        pos = R @ pose[:3, 3]

        _set_time(rr, "stamp", float(stamp))

        # A keyframe is the first frame, or once the sensor has moved
        # >= keyframe_dist since the last one (keyframe_dist <= 0 keeps all).
        moved = (
            self._last_kf_pos is None
            or float(np.linalg.norm(pos - self._last_kf_pos)) >= self._keyframe_dist
        )
        is_keyframe = self._keyframe_dist <= 0.0 or moved

        if is_keyframe:  # skip empty scans (possible after range/tag filtering)
            world = lidar_to_world(pose, self._extrinsic, scan_pts) @ R.T
            if world.shape[0] > 0:
                self._last_kf_pos = pos.copy()
                rr.log(f"world/map/{self._i:05d}", rr.Points3D(world))
                self._i += 1

        # Grow the path one short segment at a time. Re-logging the whole strip
        # every frame would re-store the full path each frame (O(n^2)); a 2-point
        # segment per frame under its own path keeps storage O(n). Color/size are
        # logged here so the path keeps its own style independent of the map,
        # which is left to the view default (editable in the UI).
        if self._traj:
            rr.log(
                f"world/trajectory/{len(self._traj):05d}",
                rr.LineStrips3D(
                    [np.array([self._traj[-1], pos])], colors=[255, 180, 40], radii=0.05
                ),
            )
        self._traj.append(pos)
        rr.log("world/sensor", rr.Transform3D(translation=pos, mat3x3=R @ pose[:3, :3], axis_length=0.5))

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
