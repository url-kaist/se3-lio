"""Run the SE3-LIO binding offline over a rosbag and write a TUM trajectory.

Reproduces the node's per-frame data path so the output is directly comparable
to the live ROS2 node's /local/odometry.
"""

import argparse
import time

import numpy as np

import bag_reader
import tum
from se3_lio import SE3LIO, load_node_params


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--params", required=True)
    ap.add_argument("--max-frames", type=int, default=300)
    ap.add_argument("--out", required=True)
    ap.add_argument("--rerun-save", help="write a Rerun .rrd recording to this path")
    ap.add_argument("--rerun-spawn", action="store_true", help="open the Rerun viewer live")
    args = ap.parse_args()

    cfg = load_node_params(args.params)
    print(f"imu_topic={cfg['imu_topic']}  lidar_topic={cfg['lidar_topic']}  "
          f"min_range={cfg['min_range']}")

    t0 = time.time()
    imus, scans = bag_reader.read_streams(
        args.bag, cfg["imu_topic"], cfg["lidar_topic"], cfg["min_range"], args.max_frames
    )
    print(f"read {len(imus)} IMU, {len(scans)} scans in {time.time() - t0:.1f}s")

    frames = bag_reader.synchronize(imus, scans, args.max_frames)
    print(f"synchronized {len(frames)} frames")

    logger = None
    if args.rerun_save or args.rerun_spawn:
        from se3_lio.viz import RerunLogger

        logger = RerunLogger(cfg["extrinsic"])

    odom = SE3LIO(cfg["config"], cfg["extrinsic"])
    stamps, poses = [], []
    t0 = time.time()
    for i, (scan, imu_block) in enumerate(frames):
        state = odom.register_frame(
            scan["pts"], scan["offsets"], imu_block, frame_stamp=scan["header_ts"]
        )
        stamps.append(state.stamp)
        poses.append(np.array(state.pose))
        if logger is not None:
            logger.log_frame(state.stamp, poses[-1], scan["pts"])
        if (i + 1) % 50 == 0:
            print(f"  frame {i + 1}/{len(frames)}  stamp={state.stamp:.3f}  "
                  f"pos={poses[-1][:3, 3]}  inliers={state.num_inliers}")
    print(f"processed {len(frames)} frames in {time.time() - t0:.1f}s")

    tum.write(args.out, stamps, poses)
    print(f"wrote {args.out}")

    if logger is not None and args.rerun_save:
        logger.save(args.rerun_save)
        print(f"wrote {args.rerun_save}")
    if logger is not None and args.rerun_spawn:
        logger.spawn()


if __name__ == "__main__":
    main()
