"""`se3_lio_pipeline` — run SE(3)-LIO over a ROS2 rosbag and write a TUM trajectory."""

import argparse
from pathlib import Path

from se3_lio.config import load_node_params
from se3_lio.datasets import RosbagDataset, Ros1BagDataset
from se3_lio.pipeline import OdometryPipeline


def _make_dataset(input_type, bag, p, max_frames):
    args = (bag, p["imu_topic"], p["lidar_topic"], p["min_range"], max_frames)
    if input_type == "ros1-ouster":
        return Ros1BagDataset(*args)
    return RosbagDataset(*args)


def run():
    ap = argparse.ArgumentParser(
        prog="se3_lio_pipeline",
        description="Run SE(3)-LIO odometry over a rosbag (ROS2/Livox or ROS1/Ouster).",
    )
    ap.add_argument("bag", help="ROS2 rosbag dir (Livox) or ROS1 .bag file (Ouster)")
    ap.add_argument(
        "--params",
        required=True,
        help="node config yaml (topics, extrinsic, noise, voxel map)",
    )
    ap.add_argument(
        "--input-type",
        choices=["auto", "ros2-livox", "ros1-ouster"],
        default="auto",
        help="input format (auto: .bag -> ros1-ouster, directory -> ros2-livox)",
    )
    ap.add_argument("--max-frames", type=int, default=None, help="limit number of frames")
    ap.add_argument(
        "--output", default="results", help="output directory for the TUM trajectory"
    )
    ap.add_argument("--no-progress", action="store_true", help="disable progress bar")
    args = ap.parse_args()

    input_type = args.input_type
    if input_type == "auto":
        input_type = "ros1-ouster" if Path(args.bag).is_file() else "ros2-livox"

    p = load_node_params(args.params)
    print(
        f"bag={args.bag}  input={input_type}\n"
        f"imu_topic={p['imu_topic']}  lidar_topic={p['lidar_topic']}  "
        f"min_range={p['min_range']}"
    )

    dataset = _make_dataset(input_type, args.bag, p, args.max_frames)
    print(f"synchronized {len(dataset)} frames")

    pipeline = OdometryPipeline(dataset, p["config"], p["extrinsic"])
    pipeline.run(progress=not args.no_progress)

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{Path(args.bag.rstrip('/')).name}_se3lio.tum"
    pipeline.save_tum(out_path)

    print(pipeline.summary())
    print(f"trajectory -> {out_path}")


if __name__ == "__main__":
    run()
