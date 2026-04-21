#ifndef DATA_TYPE_H
#define DATA_TYPE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

using PointTypePCL = pcl::PointXYZINormal;
using PointCloudTypePCL = pcl::PointCloud<pcl::PointXYZINormal>;

struct CustomPointType {
    PCL_ADD_POINT4D;
    float intensity;
    double timestamp;  // timestamp denotes (point_time - pointcloud_begin_time)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(CustomPointType,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, intensity, intensity)(double, timestamp, timestamp))

namespace se3_lio {

using PointCloudType = pcl::PointCloud<CustomPointType>;
struct Header {
    uint32_t seq;
    double timestamp;
    std::string frame_id;
};

struct IMU {
    Header header;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
};

struct LiDAR {
    Header header;
    pcl::PointCloud<CustomPointType> points;
    std::vector<Eigen::Matrix3d> noises;
};

struct GPS {
    Header header;
    Eigen::Matrix4d pose;
    Eigen::Matrix<double, 6, 6> covariance;
};

enum StateIdx {
    POSE = 0,
    POS = 0,
    ROT = 3,
    VEL = 6,
    B_G = 9,
    B_A = 12,
    GRAV = 15,
};

// Noise Cov Matrix Index
enum NoiseIdx {
    N_GYRO = 0,
    N_ACC = 3,
    N_B_GYRO = 6,
    N_B_ACC = 9,
};

#define DOF_DIM 18
#define ERR_DIM 18
#define INPUT_DIM 12
using ErrVector = Eigen::Matrix<double, ERR_DIM, 1>;
using CovMatrix = Eigen::Matrix<double, ERR_DIM, ERR_DIM>;
using MeasCovMatrix = Eigen::Matrix<double, ERR_DIM, INPUT_DIM>;

struct Measurement {
    bool is_synced;

    std::vector<IMU> imu;
    LiDAR lidar;
    LiDAR raw_lidar;     // raw lidar points
    LiDAR merged_lidar;  // merged lidar points on IMU
    GPS gps;
};

struct State {
    double stamp;
    int num_inliers;
    double residual;
    Eigen::Matrix4d pose;
    Eigen::Vector3d vel;
    Eigen::Vector3d bg;
    Eigen::Vector3d ba;
    Eigen::Vector3d grav;

    CovMatrix covariance;

    State() {
        stamp = 0.0;
        pose = Eigen::Matrix4d::Identity();
        vel = Eigen::Vector3d::Zero();
        bg = Eigen::Vector3d::Zero();
        ba = Eigen::Vector3d::Zero();
        grav = Eigen::Vector3d::Zero();
        covariance = CovMatrix::Zero();
    }

    inline Eigen::Matrix3d rot() const { return pose.block<3, 3>(0, 0); }
    inline Eigen::Vector3d pos() const { return pose.block<3, 1>(0, 3); }
    inline Eigen::Matrix<double, 6, 6> pose_cov() const { return covariance.block<6, 6>(0, 0); }
};

using MeasurementPtr = std::shared_ptr<Measurement>;
using StatePtr = std::shared_ptr<State>;

}  // namespace se3_lio

#endif  // DATA_TYPE_H