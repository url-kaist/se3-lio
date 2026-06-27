"""`se3_lio_pipeline` — run SE(3)-LIO over a ROS2 rosbag and write a TUM trajectory."""

import argparse
from pathlib import Path

from se3_lio.config import load_node_params
from se3_lio.datasets import build_dataset
from se3_lio.pipeline import OdometryPipeline


def run():
    ap = argparse.ArgumentParser(
        prog="se3_lio_pipeline",
        description="Run SE(3)-LIO odometry over a rosbag (ROS2/Livox or ROS1/Ouster).",
    )
    ap.add_argument("bag", help="ROS2 rosbag dir (Livox) or ROS1 .bag file (Ouster)")
    ap.add_argument(
        "--config",
        dest="config",
        help="dataset config yaml (extrinsic; optional topics + algorithm overrides). "
        "Unset keys fall back to the SE3LIOConfig code defaults.",
    )
    ap.add_argument("--params", dest="config", help=argparse.SUPPRESS)  # deprecated alias
    ap.add_argument("--imu-topic", help="override imu_topic from config")
    ap.add_argument("--lidar-topic", help="override lidar_topic from config")
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
    ap.add_argument(
        "--visualize", action="store_true", help="open a live Polyscope viewer (needs se3-lio[viz])"
    )
    ap.add_argument("--rerun-save", help="write a Rerun .rrd recording to this path (needs se3-lio[rerun])")
    ap.add_argument("--rerun-spawn", action="store_true", help="open the Rerun viewer live")
    ap.add_argument(
        "--rerun-keyframe-dist",
        type=float,
        default=0.0,
        help="log the world scan only every N meters of travel (0 = every frame)",
    )
    args = ap.parse_args()
    if not args.config:
        ap.error("--config is required")

    input_type = args.input_type
    if input_type == "auto":
        input_type = "ros1-ouster" if Path(args.bag).is_file() else "ros2-livox"

    p = load_node_params(args.config)
    if args.imu_topic:
        p["imu_topic"] = args.imu_topic
    if args.lidar_topic:
        p["lidar_topic"] = args.lidar_topic
    print(
        f"bag={args.bag}  input={input_type}\n"
        f"imu_topic={p['imu_topic']}  lidar_topic={p['lidar_topic']}  "
        f"min_range={p['min_range']}"
    )

    dataset = build_dataset(
        input_type, args.bag, p["imu_topic"], p["lidar_topic"], p["min_range"], args.max_frames
    )
    print(f"synchronized {len(dataset)} frames")

    logger = None
    if args.visualize:
        from se3_lio.viz import PolyscopeVisualizer

        logger = PolyscopeVisualizer(p["extrinsic"])
    elif args.rerun_save or args.rerun_spawn:
        from se3_lio.viz import RerunLogger

        logger = RerunLogger(
            p["extrinsic"],
            keyframe_dist=args.rerun_keyframe_dist,
            save_path=args.rerun_save,
        )

    pipeline = OdometryPipeline(dataset, p["config"], p["extrinsic"])
    pipeline.run(progress=not args.no_progress, logger=logger)

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{Path(args.bag.rstrip('/')).name}_se3lio.tum"
    pipeline.save_tum(out_path)

    print(pipeline.summary())
    print(f"trajectory -> {out_path}")
    if args.rerun_save:
        print(f"rerun -> {args.rerun_save}")

    if args.visualize and logger is not None:
        logger.hold()
    if args.rerun_spawn and logger is not None:
        logger.spawn()


if __name__ == "__main__":
    run()
