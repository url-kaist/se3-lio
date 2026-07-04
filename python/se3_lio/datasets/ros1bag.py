"""ROS1 rosbag dataset (Ouster PointCloud2) for SE(3)-LIO.

Reproduces the ROS1 node's data path:
  - ros1_conversion.h::convertIMUMessage / convertOusterMessage
  - MeasurementSynchronizer::synchronizeIMULiDAR (shared core, ported in rosbag.py)

ROS1 `.bag` files are read with the `rosbags` library (pure Python, no ROS1
install needed). Ouster clouds are `sensor_msgs/PointCloud2` with per-point `t`
(uint32, ns offset from scan start).
"""

from pathlib import Path

import numpy as np

from se3_lio.datasets.rosbag import Frame, synchronize

# sensor_msgs/PointField datatype -> numpy little-endian format
_PF_NP = {1: "<i1", 2: "<u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}


def _stamp(header):
    """Header stamp in seconds (ROS2 ``nanosec`` or ROS1 ``nsec``)."""
    s = header.stamp
    ns = getattr(s, "nanosec", None)
    if ns is None:
        ns = getattr(s, "nsec", 0)
    return s.sec + ns * 1e-9


def _pc2_arrays(msg, needed):
    """Structured view of a PointCloud2 over `needed` fields (zero-copy)."""
    fmt = {f.name: (f.offset, _PF_NP[f.datatype]) for f in msg.fields if f.name in needed}
    dt = np.dtype(
        {
            "names": list(fmt),
            "formats": [fmt[n][1] for n in fmt],
            "offsets": [fmt[n][0] for n in fmt],
            "itemsize": msg.point_step,
        }
    )
    return np.frombuffer(bytes(msg.data), dtype=dt)


def _xyz_keep(arr, min_range):
    xyz = np.stack(
        [arr["x"].astype(np.float64), arr["y"].astype(np.float64), arr["z"].astype(np.float64)],
        axis=1,
    )
    # Range test in float32 to match the ROS node bit-for-bit: CustomPointType is
    # float, so the C++ converters compute x*x+y*y+z*z in float32. Doing it in
    # float64 here flips 1-2 boundary points per scan, which a near-unstable run
    # amplifies into trajectory divergence. Cast x/y/z (already float32) and keep
    # the float32 sum's order, then promote for the compare (NaN-safe). [[tartandrive-dataset-setup]]
    xf, yf, zf = arr["x"], arr["y"], arr["z"]
    d2 = (xf * xf + yf * yf + zf * zf).astype(np.float64)
    keep = d2 > (min_range * min_range)
    return xyz, keep


def _convert_ouster(msg, min_range):
    """Port of convertOusterMessage: per-point time is the `t` field (ns offset
    from scan start). Original order preserved (so the last kept point matches
    C++ `points.back()` used by the synchronizer)."""
    arr = _pc2_arrays(msg, ("x", "y", "z", "intensity", "t"))
    xyz, keep = _xyz_keep(arr, min_range)
    offs = arr["t"].astype(np.float64) * 1e-9
    return xyz[keep], offs[keep], 0.0


def _convert_hesai(msg, min_range):
    """Port of convertHesaiMessage: per-point `timestamp` is absolute seconds,
    so the offset from scan start is timestamp - header stamp (GrandTour Hesai)."""
    arr = _pc2_arrays(msg, ("x", "y", "z", "intensity", "timestamp"))
    xyz, keep = _xyz_keep(arr, min_range)
    offs = arr["timestamp"].astype(np.float64) - _stamp(msg.header)
    return xyz[keep], offs[keep], 0.0


def _convert_velodyne(msg, min_range):
    """Port of convertVelodyneMessage: per-point `time` (float32 seconds) is
    relative to the header stamp (scan end), in [-period, 0]. Shift by the
    minimum time so offsets are ascending and >= 0 and the header marks the scan
    start; t0 is taken over all points (before filtering) for binding parity."""
    arr = _pc2_arrays(msg, ("x", "y", "z", "intensity", "time"))
    xyz, keep = _xyz_keep(arr, min_range)
    t = arr["time"].astype(np.float64)
    header_shift = float(t.min()) if t.size else 0.0
    offs = t - header_shift
    return xyz[keep], offs[keep], header_shift


def _convert_lidar(msg, min_range):
    """Dispatch by per-point time field, returning (xyz, offsets, header_shift):
    Ouster has `t` (uint32 ns), Hesai/GrandTour has `timestamp` (float64 absolute
    seconds), Velodyne has `time` (float32 seconds relative to scan end). The
    header shift is added to the scan stamp so it marks the scan start."""
    names = {f.name for f in msg.fields}
    if "t" in names:
        return _convert_ouster(msg, min_range)
    if "timestamp" in names:
        return _convert_hesai(msg, min_range)
    if "time" in names:
        return _convert_velodyne(msg, min_range)
    raise RuntimeError(f"no per-point time field (t/timestamp/time) in cloud: {sorted(names)}")


def _as_bag_list(bag):
    """Resolve a bag spec to a list of Paths. Accepts a single path, a list of
    paths, or a glob string. Multiple bags (e.g. LiDAR + IMU split across files,
    or time chunks) are merged in log-time order by AnyReader."""
    import glob

    if not isinstance(bag, (str, Path)):
        return [Path(b) for b in bag]
    s = str(bag)
    if any(c in s for c in "*?["):
        matches = sorted(glob.glob(s))
        if not matches:
            raise FileNotFoundError(f"no bags match glob: {s}")
        return [Path(m) for m in matches]
    return [Path(s)]


def read_streams(bag_path, imu_topic, lidar_topic, min_range, max_scans=None):
    from rosbags.highlevel import AnyReader

    imus = []
    scans = []
    margin = 2
    with AnyReader(_as_bag_list(bag_path)) as reader:
        conns = [c for c in reader.connections if c.topic in (imu_topic, lidar_topic)]
        if not conns:
            raise RuntimeError(
                f"topics not found in bag: imu={imu_topic} lidar={lidar_topic}"
            )
        for conn, _t, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            if conn.topic == imu_topic:
                imus.append(
                    [
                        _stamp(m.header),
                        m.linear_acceleration.x,
                        m.linear_acceleration.y,
                        m.linear_acceleration.z,
                        m.angular_velocity.x,
                        m.angular_velocity.y,
                        m.angular_velocity.z,
                    ]
                )
            else:
                pts, offs, header_shift = _convert_lidar(m, min_range)
                scans.append({"header_ts": _stamp(m.header) + header_shift, "pts": pts, "offsets": offs})
                if max_scans and len(scans) >= max_scans + margin:
                    break
    return np.array(imus, dtype=np.float64), scans


class Ros1BagDataset:
    """Iterable of synchronized SE(3)-LIO `Frame`s from a ROS1 (.bag) Ouster sequence."""

    def __init__(self, bag_path, imu_topic, lidar_topic, min_range, max_frames=None):
        imus, scans = read_streams(bag_path, imu_topic, lidar_topic, min_range, max_frames)
        self._frames = synchronize(imus, scans, max_frames)

    def __len__(self):
        return len(self._frames)

    def __iter__(self):
        for scan, imu_block in self._frames:
            yield Frame(
                points=scan["pts"],
                point_times=scan["offsets"],
                imu=imu_block,
                stamp=scan["header_ts"],
            )
