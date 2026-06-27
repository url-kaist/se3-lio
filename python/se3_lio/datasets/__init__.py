from se3_lio.datasets.rosbag import RosbagDataset, Frame
from se3_lio.datasets.ros1bag import Ros1BagDataset

__all__ = ["RosbagDataset", "Ros1BagDataset", "Frame", "build_dataset"]


def build_dataset(input_type, bag, imu_topic, lidar_topic, min_range, max_frames=None):
    """Construct the dataset reader for ``input_type``.

    ``ros1-ouster`` -> Ros1BagDataset (ROS1 .bag, Ouster PointCloud2); anything
    else -> RosbagDataset (ROS2 rosbag directory, Livox CustomMsg).
    """
    args = (bag, imu_topic, lidar_topic, min_range, max_frames)
    if input_type == "ros1-ouster":
        return Ros1BagDataset(*args)
    return RosbagDataset(*args)
