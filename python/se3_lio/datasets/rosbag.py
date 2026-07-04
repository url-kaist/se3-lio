"""ROS2 rosbag dataset for SE(3)-LIO.

Reproduces the ROS2 node's data path exactly so a Python run matches the node:
  - ros2_conversion.h::convertIMUMessage / convertLivoxMessage
  - MeasurementSynchronizer::synchronizeIMULiDAR

Reading uses rosbag2_py + the message Python modules, so it must run in an
environment where ROS2 (and livox_ros_driver2) is sourced — the same env the
binding is built in.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Frame:
    points: np.ndarray  # (N, 3) xyz in the LiDAR frame
    point_times: np.ndarray  # (N,) per-point time offset from frame start [s]
    imu: np.ndarray  # (M, 7) rows of [t, ax, ay, az, gx, gy, gz]
    stamp: float  # absolute scan start time [s]


def _open_reader(bag_path):
    import rosbag2_py

    storage = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    conv = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, conv)
    return reader


def _convert_livox(msg, min_range):
    """Port of convertLivoxMessage: returns (pts Nx3 float64, offsets N float64)."""
    pts = msg.points
    n = len(pts)
    min_r2 = min_range * min_range
    last_x = last_y = last_z = 0.0
    xs, ys, zs, offs = [], [], [], []
    for i in range(n):  # keep the first point too (mirrors the node's i=0 fix)
        pt = pts[i]
        tag = pt.tag & 0x30
        if tag != 0x10 and tag != 0x00:
            continue
        x, y, z = pt.x, pt.y, pt.z
        moved = (
            abs(x - last_x) > 1e-7 or abs(y - last_y) > 1e-7 or abs(z - last_z) > 1e-7
        )
        if moved and (x * x + y * y + z * z > min_r2):
            offset = pt.offset_time * 1e-9
            if offset > 0.1:
                continue  # skip; last_* is NOT updated (matches C++ `continue`)
            xs.append(x)
            ys.append(y)
            zs.append(z)
            offs.append(offset)
        last_x, last_y, last_z = x, y, z
    pts_arr = np.array([xs, ys, zs], dtype=np.float64).T.reshape(-1, 3)
    return pts_arr, np.array(offs, dtype=np.float64)


def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def read_streams(bag_path, imu_topic, lidar_topic, min_range, max_scans=None):
    """Read until `max_scans` (+margin) LiDAR scans are collected, with all IMU seen."""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu
    from livox_ros_driver2.msg import CustomMsg

    reader = _open_reader(bag_path)
    imus = []  # rows of [t, ax, ay, az, gx, gy, gz]
    scans = []  # dicts: {header_ts, pts, offsets}
    margin = 2  # extra scans so IMU coverage past the last kept scan is present
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == imu_topic:
            m = deserialize_message(data, Imu)
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
        elif topic == lidar_topic:
            m = deserialize_message(data, CustomMsg)
            pts, offs = _convert_livox(m, min_range)
            scans.append({"header_ts": _stamp(m.header), "pts": pts, "offsets": offs})
            if max_scans and len(scans) >= max_scans + margin:
                break
    return np.array(imus, dtype=np.float64), scans


def synchronize(imus, scans, max_frames=None):
    """Port of MeasurementSynchronizer::synchronizeIMULiDAR.

    Returns (scan, imu_block) frames; IMU is partitioned non-overlapping across scans.
    The batch pass is just the online synchronizer fed everything at once, so the
    two stay bit-for-bit equivalent (single source of the sync rule).
    """
    from se3_lio.online_sync import OnlineSynchronizer

    sync = OnlineSynchronizer()
    for row in imus:
        sync.add_imu(row)
    for scan in scans:
        sync.add_scan(scan["header_ts"], scan["pts"], scan["offsets"])
    frames = sync.drain()
    if max_frames:
        frames = frames[:max_frames]
    return frames


class RosbagDataset:
    """Iterable of synchronized SE(3)-LIO `Frame`s from a ROS2 rosbag."""

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
