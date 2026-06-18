# SE(3)-LIO Python bindings

Python bindings for the ROS-agnostic SE(3) LiDAR-Inertial Odometry C++ core.

## Build

The bindings link the C++ core, which requires PCL, Eigen, Sophus and OpenMP.
Build inside the project's ROS2 Docker image (`docker/ros2/`), where these are
available.

```bash
# inside the container, from the repo root
pip install --no-build-isolation -ve ./python/   # requires: pip install pybind11 scikit-build-core
# or, with build isolation (pulls build deps automatically):
pip install -ve ./python/
```

## Usage (high-level API)

```python
import numpy as np
from se3_lio import SE3LIO, SE3LIOConfig, load_node_params

config = SE3LIOConfig(downsample_resolution=0.5, max_iter=4, verbose=False)
odom = SE3LIO(config, lidar_extrinsic=np.eye(4))

# or load the ROS2 node's params.yaml (same mapping as lio_node.cpp):
params = load_node_params("pipelines/ros2/config/params.yaml")
odom = SE3LIO(params["config"], params["extrinsic"])

# points:      (N, 3) float xyz in the LiDAR frame
# point_times: (N,)   float per-point time offset from frame start [s]
# imu:         (M, 7) float rows of [t, ax, ay, az, gx, gy, gz]
state = odom.register_frame(points, point_times, imu, frame_stamp)
print(state.pose, state.vel, state.covariance)
```

`register_frame` mirrors the per-frame data path of the ROS2 node: it builds a
synced measurement, applies the LiDAR extrinsic, sorts points by relative
timestamp, and runs one `estimatePose` step. Set `verbose=True` to enable the
core's per-frame stdout logging (off by default).

## CLI pipeline

`se3_lio_pipeline` runs odometry over a rosbag and writes a TUM trajectory — the
Python equivalent of the node, offline. The input format is auto-detected:

| input | format | LiDAR | reader |
|-------|--------|-------|--------|
| rosbag2 directory | ROS2 | Livox `CustomMsg` | `rosbag2_py` |
| `*.bag` file | ROS1 | Ouster `PointCloud2` | `rosbags` |

```bash
# ROS2 / Livox (rosbag2 directory)
se3_lio_pipeline <rosbag_dir> --params pipelines/ros2/config/params.yaml \
    --max-frames 300 --output results/

# ROS1 / Ouster (e.g. NTU VIRAL .bag)
se3_lio_pipeline eee_01.bag --params pipelines/ros1/config/ntu.yaml \
    --max-frames 1500 --output results/
# -> results/<bag>_se3lio.tum  + a summary (frames, path length, final pose)
```

Use `--input-type ros2-livox|ros1-ouster` to override auto-detection. It reads
the bag, reproduces the node's conversion + synchronization
([rosbag.py](se3_lio/datasets/rosbag.py) / [ros1bag.py](se3_lio/datasets/ros1bag.py)),
and runs [OdometryPipeline](se3_lio/pipeline.py) over the frames. Use the same
pieces programmatically:

```python
from se3_lio import load_node_params
from se3_lio.datasets import RosbagDataset
from se3_lio.pipeline import OdometryPipeline

p = load_node_params("pipelines/ros2/config/params.yaml")
dataset = RosbagDataset(bag_dir, p["imu_topic"], p["lidar_topic"], p["min_range"])
pipeline = OdometryPipeline(dataset, p["config"], p["extrinsic"]).run()
pipeline.save_tum("traj.tum")
print(pipeline.summary())
```

## Visualization (Rerun)

`se3_lio_pipeline` accepts `--rerun-save <path.rrd>`
(and `--rerun-spawn` to open the viewer live). Per frame the estimated pose, the
scan transformed into the world frame, and the trajectory are logged; Rerun
accumulates the world scans across the timeline, so playing it back shows the map
being built.

```bash
pip install -e ./python/[viz]          # adds rerun-sdk
se3_lio_pipeline eee_01.bag --params pipelines/ros1/config/ntu.yaml \
    --max-frames 2500 --output results/ --rerun-save results/eee_01.rrd
rerun results/eee_01.rrd               # open the recording (match the rerun-sdk version)
```

The logger lives in [se3_lio/viz/rerun_logger.py](se3_lio/viz/rerun_logger.py)
(`lidar_to_world`, `RerunLogger`, `read_rrd_trajectory`). `tests/test_rerun_viz.py`
covers it (transform math + `.rrd` round-trip, runnable without the binding).
Current logging uses the raw input scan;
logging the undistorted + downsampled cloud needs exposing it from the core
(planned next).

### Raw binding

The `_`-prefixed names (`se3_lio_pybind._SE3LIO`, `_SE3LIOConfig`, `_State`) are
the raw binding the high-level API is built on.

```python
import numpy as np
from se3_lio import se3_lio_pybind as lio

config = lio._SE3LIOConfig()
config.downsample_resolution = 0.5
config.max_iter = 4

odom = lio._SE3LIO(config, np.eye(4))  # second arg: 4x4 LiDAR->body extrinsic

# points:      (N, 3) float64 xyz in the LiDAR frame
# point_times: (N,)   float64 per-point time offset from frame start [s]
# imu:         (M, 7) float64 rows of [t, ax, ay, az, gx, gy, gz]
state = odom._register_frame(points, point_times, imu, frame_stamp)

print(state.pose)        # 4x4 SE(3) pose
print(state.vel, state.bg, state.ba, state.grav)
print(state.covariance)  # 18x18
```
