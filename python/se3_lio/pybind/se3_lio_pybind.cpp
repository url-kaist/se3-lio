#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <memory>

#include "common/data_type.h"
#include "common/utils.h"
#include "pipeline/SE3_LIO.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace {

// Mirrors the per-frame data path of the ROS2 node (lio_node.cpp::process):
// build a synced Measurement, apply the LiDAR extrinsic, sort points by
// relative timestamp, then run a single estimatePose step.
class SE3LIOWrapper {
public:
    SE3LIOWrapper(const se3_lio::pipeline::SE3_LIO_Config &config,
                  const Eigen::Matrix4d &lidar_extrinsic)
        : pipeline_(config), extrinsic_(lidar_extrinsic) {}

    se3_lio::State RegisterFrame(
        const py::array_t<double, py::array::c_style | py::array::forcecast> &points,
        const py::array_t<double, py::array::c_style | py::array::forcecast> &point_times,
        const py::array_t<double, py::array::c_style | py::array::forcecast> &imu,
        double frame_stamp) {
        if (points.ndim() != 2 || points.shape(1) != 3)
            throw std::invalid_argument("points must have shape (N, 3)");
        if (point_times.ndim() != 1 || point_times.shape(0) != points.shape(0))
            throw std::invalid_argument("point_times must have shape (N,) matching points");
        if (imu.ndim() != 2 || imu.shape(1) != 7)
            throw std::invalid_argument("imu must have shape (M, 7): [t, ax, ay, az, gx, gy, gz]");

        auto meas = std::make_shared<se3_lio::Measurement>();
        meas->is_synced = true;

        auto imu_u = imu.unchecked<2>();
        meas->imu.reserve(imu_u.shape(0));
        for (py::ssize_t i = 0; i < imu_u.shape(0); ++i) {
            se3_lio::IMU sample;
            sample.header.timestamp = imu_u(i, 0);
            sample.linear_acceleration =
                Eigen::Vector3d(imu_u(i, 1), imu_u(i, 2), imu_u(i, 3));
            sample.angular_velocity = Eigen::Vector3d(imu_u(i, 4), imu_u(i, 5), imu_u(i, 6));
            meas->imu.push_back(sample);
        }

        meas->lidar.header.timestamp = frame_stamp;
        auto pts = points.unchecked<2>();
        auto times = point_times.unchecked<1>();
        meas->lidar.points.reserve(pts.shape(0));
        for (py::ssize_t i = 0; i < pts.shape(0); ++i) {
            CustomPointType point;
            point.x = static_cast<float>(pts(i, 0));
            point.y = static_cast<float>(pts(i, 1));
            point.z = static_cast<float>(pts(i, 2));
            point.intensity = 0.0f;
            point.timestamp = times(i);
            meas->lidar.points.push_back(point);
        }

        meas->lidar.points = se3_lio::transformPointCloud(meas->lidar.points, extrinsic_);
        std::sort(meas->lidar.points.begin(), meas->lidar.points.end(),
                  [](const CustomPointType &a, const CustomPointType &b) {
                      return a.timestamp < b.timestamp;
                  });

        se3_lio::MeasurementPtr meas_ptr = meas;
        pipeline_.estimatePose(meas_ptr);
        return pipeline_.getState();
    }

    Eigen::Matrix4d LastPose() { return pipeline_.getState().pose; }

private:
    se3_lio::pipeline::SE3_LIO pipeline_;
    Eigen::Matrix4d extrinsic_;
};

}  // namespace

PYBIND11_MODULE(se3_lio_pybind, m) {
    using se3_lio::State;
    using Config = se3_lio::pipeline::SE3_LIO_Config;

    py::class_<Config>(m, "_SE3LIOConfig")
        .def(py::init<>())
        .def_readwrite("acc_noise", &Config::acc_noise)
        .def_readwrite("gyr_noise", &Config::gyr_noise)
        .def_readwrite("bg_noise", &Config::bg_noise)
        .def_readwrite("ba_noise", &Config::ba_noise)
        .def_readwrite("lidar_range_noise", &Config::lidar_range_noise)
        .def_readwrite("lidar_angle_noise", &Config::lidar_angle_noise)
        .def_readwrite("downsample_resolution", &Config::downsample_resolution)
        .def_readwrite("max_iter", &Config::max_iter)
        .def_readwrite("voxel_map_resolution", &Config::voxel_map_resolution)
        .def_readwrite("voxel_map_max_layer", &Config::voxel_map_max_layer)
        .def_readwrite("voxel_map_layer_size", &Config::voxel_map_layer_size)
        .def_readwrite("voxel_map_max_point_size", &Config::voxel_map_max_point_size)
        .def_readwrite("voxel_map_plane_thres", &Config::voxel_map_plane_thres)
        .def_readwrite("verbose", &Config::verbose);

    py::class_<State>(m, "_State")
        .def(py::init<>())
        .def_readonly("stamp", &State::stamp)
        .def_readonly("num_inliers", &State::num_inliers)
        .def_readonly("residual", &State::residual)
        .def_readonly("pose", &State::pose)
        .def_readonly("vel", &State::vel)
        .def_readonly("bg", &State::bg)
        .def_readonly("ba", &State::ba)
        .def_readonly("grav", &State::grav)
        .def_readonly("covariance", &State::covariance);

    py::class_<SE3LIOWrapper>(m, "_SE3LIO")
        .def(py::init<const Config &, const Eigen::Matrix4d &>(), "config"_a,
             "lidar_extrinsic"_a)
        .def("_register_frame", &SE3LIOWrapper::RegisterFrame, "points"_a, "point_times"_a,
             "imu"_a, "frame_stamp"_a)
        .def("_last_pose", &SE3LIOWrapper::LastPose);
}
