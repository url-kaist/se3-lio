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

_NEEDED = ("x", "y", "z", "intensity", "t")


def _pc2_arrays(msg):
    """Structured view of a PointCloud2 over the fields we need (zero-copy)."""
    fmt = {f.name: (f.offset, _PF_NP[f.datatype]) for f in msg.fields if f.name in _NEEDED}
    dt = np.dtype(
        {
            "names": list(fmt),
            "formats": [fmt[n][1] for n in fmt],
            "offsets": [fmt[n][0] for n in fmt],
            "itemsize": msg.point_step,
        }
    )
    return np.frombuffer(bytes(msg.data), dtype=dt)


def _convert_ouster(msg, min_range):
    """Port of convertOusterMessage: returns (pts Nx3 float64, offsets N float64).

    Keeps points with range > min_range (original order preserved, so the last
    kept point matches C++ `points.back()` used by the synchronizer).
    """
    arr = _pc2_arrays(msg)
    xyz = np.stack(
        [arr["x"].astype(np.float64), arr["y"].astype(np.float64), arr["z"].astype(np.float64)],
        axis=1,
    )
    offs = arr["t"].astype(np.float64) * 1e-9
    r2 = (xyz * xyz).sum(axis=1)
    keep = r2 > (min_range * min_range)  # NaN-safe: NaN comparisons are False
    return xyz[keep], offs[keep]


def _stamp(header):
    s = header.stamp
    ns = getattr(s, "nanosec", None)
    if ns is None:
        ns = getattr(s, "nsec", 0)
    return s.sec + ns * 1e-9


def read_streams(bag_path, imu_topic, lidar_topic, min_range, max_scans=None):
    from rosbags.highlevel import AnyReader

    imus = []
    scans = []
    margin = 2
    with AnyReader([Path(bag_path)]) as reader:
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
                pts, offs = _convert_ouster(m, min_range)
                scans.append({"header_ts": _stamp(m.header), "pts": pts, "offsets": offs})
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
