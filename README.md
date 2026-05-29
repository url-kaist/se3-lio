<div align="center">
    <h1>SE(3)-LIO</h1>
    <img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" />
    <img src="https://img.shields.io/badge/ROS1-Noetic-blue" />
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

> 🚧 This repository is under active construction. The ROS1 pipeline builds and
> runs; datasets, example configs, and a project page are on the way.

## :open_file_folder: Layout

SE(3)-LIO is a ROS1 node built on a ROS-agnostic C++ core:

- `cpp/se3_lio/` — ROS-agnostic C++ core (state estimation, map management, pipeline)
- `pipelines/ros1/` — ROS1 (Noetic, catkin) node, launch, config, rviz
- `docker/ros1/` — Docker setup for building and running

______________________________________________________________________

## :package: Build

All build and run steps run **inside Docker**. On the host you only build the
image and drop into the container.

```bash
# host
bash docker/ros1/build_docker.sh          # builds se3_lio:ros1
bash docker/ros1/run_docker.sh      # drops into /ws inside the container

# inside the container
cd /ws
catkin build se3_lio
source devel/setup.bash
```

______________________________________________________________________

## :rocket: Run

```bash
# replaying a rosbag
roslaunch se3_lio run_se3lio_ntu.launch use_sim_time:=true
rosbag play --clock <data.bag>
```

`use_sim_time:=true` makes the node follow the bag clock (`rosbag play --clock`
publishes `/clock`). For live sensors, omit it — the default is `false`.

### Configuration

Parameters live in [pipelines/ros1/config/ntu.yaml](pipelines/ros1/config/ntu.yaml).
Swap in a dataset-specific yaml by editing the `<rosparam … file=…/>` line in
[pipelines/ros1/launch/run_se3lio_ntu.launch](pipelines/ros1/launch/run_se3lio_ntu.launch).

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
