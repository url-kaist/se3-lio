<div align="center">
    <h1>SE(3)-LIO</h1>
    <img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" />
    <img src="https://img.shields.io/badge/ROS1-Noetic-blue" />
    <img src="https://img.shields.io/badge/ROS2-Humble-blue" />
    <img src="https://img.shields.io/badge/Ubuntu-E95420?logo=ubuntu&logoColor=white" />
    <img src="https://img.shields.io/badge/License-GPL--2.0-green" />
    <br />
    <a href="https://arxiv.org/abs/2603.16118"><img src="https://img.shields.io/badge/arXiv-b33737?logo=arXiv" /></a>
    <a href="https://youtu.be/xu2S5Q_2fpc"><img src="https://img.shields.io/badge/YouTube-FF0000?logo=youtube&logoColor=white" /></a>
    <a href="https://se3-lio.github.io/"><img src="https://img.shields.io/badge/Project_Page-2ea44f" /></a>
    <br />
    <br />
    <p><strong><em>Smooth IMU propagation with jointly distributed poses on the SE(3) manifold for accurate and robust LiDAR-Inertial Odometry.</em></strong></p>
</div>

______________________________________________________________________

## :open_file_folder: Layout

SE(3)-LIO ships as ROS1 and ROS2 nodes built on a shared ROS-agnostic C++ core:

- `cpp/se3_lio/` — ROS-agnostic C++ core (state estimation, map management, pipeline)
- `pipelines/ros1/` — ROS1 (Noetic, catkin) node, launch, config, rviz
- `pipelines/ros2/` — ROS2 (Humble, ament/colcon) node, launch, config, rviz
- `docker/ros1/`, `docker/ros2/` — Docker setup for building and running each

______________________________________________________________________

## :package: Build & Run

All build and run steps run **inside Docker** — on the host you only build the
image and drop into the container.

<details>
<summary><b>ROS 1 (Noetic)</b></summary>

### Build

```bash
# host
bash docker/ros1/build_docker.sh     # builds se3_lio:ros1
bash docker/ros1/run_docker.sh       # drops into /ws inside the container

# inside the container
cd /ws
catkin build se3_lio                 # default ouster; -DLIDAR_TYPE=livox|hesai to switch
source devel/setup.bash
```

The LiDAR input type is a **build option** — pass
`--cmake-args -DLIDAR_TYPE=ouster|hesai|livox` (default `ouster`). No source edits
needed; `ouster`/`hesai` use `PointCloud2`, `livox` uses Livox `CustomMsg`.

### Run

```bash
roslaunch se3_lio run_se3lio_ntu.launch use_sim_time:=true
rosbag play --clock <data.bag>
```

`use_sim_time:=true` makes the node follow the bag clock (`rosbag play --clock`
publishes `/clock`). For live sensors, omit it — the default is `false`.
Config lives in [pipelines/ros1/config/ntu.yaml](pipelines/ros1/config/ntu.yaml);
swap datasets by editing the `<rosparam … file=…/>` line in the launch file.
The launch opens RViz automatically — it needs a display: launch via
`run_docker.sh` and run `xhost +local:root` on the host.

</details>

<details>
<summary><b>ROS 2 (Humble)</b></summary>

### Build

```bash
# host
bash docker/ros2/build_docker.sh     # builds se3_lio:ros2
bash docker/ros2/run_docker.sh       # drops into /ws inside the container

# inside the container
cd /ws
colcon build --symlink-install --packages-select se3_lio   # default livox
source install/setup.bash
```

The LiDAR input type is a **build option** — pass
`--cmake-args -DLIDAR_TYPE=ouster|livox` (default `livox`). No source edits needed;
`ouster` uses `PointCloud2`, `livox` uses Livox `CustomMsg`.

### Run

```bash
ros2 launch se3_lio run.launch.py    # add rviz:=true for visualization
ros2 bag play <rosbag2_dir>
```

Config lives in [pipelines/ros2/config/params.yaml](pipelines/ros2/config/params.yaml)
(edit `imu_topic` / `lidar_topic` / extrinsics for your sensor rig).

`rviz:=true` opens RViz (registered cloud + trajectory). RViz needs a display:
launch via `run_docker.sh` and run `xhost +local:root` on the host.

</details>

<details>
<summary><b>Python bindings (pybind11)</b></summary>

The core is exposed to Python via pybind11 ([python/](python/)). It links the C++
core (PCL, Eigen, Sophus, OpenMP), so build it **inside the ROS2 Docker image**.

### Build

```bash
# inside the se3_lio:ros2 container
pip install pybind11 scikit-build-core pydantic
pip install --no-build-isolation ./python/      # self-contained wheel ($ORIGIN RPATH)
pytest python/tests/
```

### Use

```python
from se3_lio import SE3LIO, SE3LIOConfig, load_node_params

p = load_node_params("pipelines/ros2/config/params.yaml")   # same mapping as the node
odom = SE3LIO(p["config"], p["extrinsic"])

# points (N,3) · point_times (N,) offsets [s] · imu (M,7) [t,ax,ay,az,gx,gy,gz]
state = odom.register_frame(points, point_times, imu, frame_stamp)
print(state.pose, state.vel, state.covariance)
```

`register_frame` reproduces the node's per-frame data path (build measurement →
apply extrinsic → sort by timestamp → `estimatePose`).

### CLI pipeline

`se3_lio_pipeline` runs odometry over a rosbag and writes a TUM trajectory — the
offline Python equivalent of the node. Input is auto-detected: a rosbag2
directory → ROS2/Livox, a `*.bag` file → ROS1/Ouster.

```bash
# ROS2 / Livox
se3_lio_pipeline <rosbag_dir> --params pipelines/ros2/config/params.yaml --max-frames 300
# ROS1 / Ouster (e.g. NTU VIRAL)
se3_lio_pipeline eee_01.bag --params pipelines/ros1/config/ntu.yaml --max-frames 1500
```

Add `--rerun-save <path.rrd>` (needs `pip install -e ./python/[viz]`) to also record
the trajectory and the accumulating world-frame map for [Rerun](https://rerun.io).

See [python/README.md](python/README.md) for the full API and supported inputs.

</details>

______________________________________________________________________

## :gear: Configuration

The same parameter groups apply to both ROS1 and ROS2 (in their respective
`config/` yaml):

| Group | Key | Meaning |
|-------|-----|---------|
| topics | `imu_topic`, `lidar_topic` | input topic names |
| `sensors/imu` | `acc_cov`, `gyr_cov`, `b_acc_cov`, `b_gyr_cov` | IMU noise / bias covariances |
| `sensors/lidar` | `min_range`, `range_cov`, `angle_cov` | LiDAR range gate and measurement noise |
| `sensors` | `t_exts`, `q_exts` | LiDAR→IMU extrinsic (translation, quaternion `w,x,y,z`) |
| `downsample` | `resolution` | input downsample voxel size (m) |
| | `max_iter` | max iterations per update |
| `voxel_map` | `resolution`, `max_layer`, `layer_size`, `max_point_size`, `plane_threshold` | voxel map structure and plane fitting |

______________________________________________________________________

## :construction: Roadmap

- [x] ROS2 (Humble) support
- [x] Python bindings (pybind11) + `se3_lio_pipeline` CLI (ROS2/Livox, ROS1/Ouster); PyPI distribution pending
- [ ] More dataset configs and example launches
- [ ] Continuous integration (build checks)

______________________________________________________________________

## :scroll: Citation

If you use SE(3)-LIO in your academic work, please cite our
[paper](https://arxiv.org/abs/2603.16118):

```bibtex
@inproceedings{shin2026icra,
  author    = {Shin, Gunhee and Lee, Seungjae and Kong, Jei and Seo, Youngwoo and Myung, Hyun},
  title     = {SE(3)-LIO: Smooth IMU Propagation With Jointly Distributed Poses on SE(3) Manifold for Accurate and Robust LiDAR-Inertial Odometry},
  booktitle = {Proc. of the IEEE Int. Conf. on Robotics and Automation (ICRA)},
  year      = {2026},
}
```

______________________________________________________________________

## :pray: Acknowledgements

- The voxel map ([cpp/se3_lio/core/temporary/voxel_map_util.{h,cpp}](cpp/se3_lio/core/temporary/voxel_map_util.h))
  is adapted from [HKU-MARS VoxelMap](https://github.com/hku-mars/VoxelMap) (GPL-2.0).

______________________________________________________________________

## License

This project is released under **GPL-2.0**, inherited from the bundled
[HKU-MARS VoxelMap](https://github.com/hku-mars/VoxelMap) component. See [LICENSE](LICENSE).
