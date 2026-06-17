#include "map_management.h"

namespace se3_lio {

inline std::vector<pointWithCov> transformGlobalPointsWithNoise(
    const LiDAR &_cloud,
    const Eigen::Matrix4d &_pose,
    const Eigen::Matrix<double, 6, 6> &_pose_cov) {
    Eigen::Matrix3d rot = _pose.block<3, 3>(0, 0);
    Eigen::Vector3d tls = _pose.block<3, 1>(0, 3);

    std::vector<pointWithCov> pv_list(_cloud.points.size());
    for (size_t i = 0; i < _cloud.points.size(); i++) {
        CustomPointType point = _cloud.points[i];

        Eigen::Vector3d pt = point.getVector3fMap().cast<double>();
        Eigen::Matrix3d noise = _cloud.noises[i];

        Eigen::Vector3d transformed_pt = rot * pt + tls;

        Eigen::Matrix<double, 3, 6> J_pt;
        J_pt << rot, -rot * skew_sym(pt);
        Eigen::Matrix3d transformed_noise =
            J_pt * _pose_cov * J_pt.transpose() + rot * noise * rot.transpose();

        pv_list[i].point << transformed_pt;
        pv_list[i].cov = transformed_noise;
    }

    return pv_list;
}

ManageMap::ManageMap(ManageMapConfig _config) : config_(_config) {
    is_initialized_ = false;

    if (config_.verbose) {
        std::cout << " ManageMap parameters check: " << std::endl;
        std::cout << "   resolution: " << config_.resolution << std::endl;
        std::cout << "   max_layer: " << config_.max_layer << std::endl;
        std::cout << "   layer_size: ";
        for (const auto &size : config_.layer_size) {
            std::cout << size << " ";
        }
        std::cout << std::endl;
        std::cout << "   plane_thres: " << config_.plane_thres << std::endl;
    }
}

void ManageMap::setMeasurement(LiDAR _measurement) { undistorted_cloud_ = _measurement; }

void ManageMap::setState(State _state) { curr_state_ = _state; };

void ManageMap::initMap() {
    pv_list_ = transformGlobalPointsWithNoise(undistorted_cloud_, curr_pose_, curr_pose_cov_);

    buildVoxelMap(pv_list_, config_.resolution, config_.max_layer, config_.layer_size,
                  config_.max_point_size, config_.max_point_size, config_.plane_thres, voxel_map_);

    is_initialized_ = true;

    return;
}

std::vector<Plane> ManageMap::getPlanes() {
    std::vector<Plane> planes;
    for (const auto &voxel : voxel_map_) {
        VOXEL_LOC key = voxel.first;
        Eigen::Vector3d voxel_center(key.x * config_.resolution, key.y * config_.resolution,
                                     key.z * config_.resolution);

        // if (voxel_center.norm() > config_.pub_voxelmap_max_range) continue;

        GetUpdatePlane(voxel.second, config_.max_layer, planes);
    }

    return planes;
}

void ManageMap::updateMap() {
    curr_pose_ = curr_state_.pose;
    curr_pose_cov_ = curr_state_.pose_cov();

    if (!is_initialized_) {
        initMap();
        return;
    }

    pv_list_ = transformGlobalPointsWithNoise(undistorted_cloud_, curr_pose_, curr_pose_cov_);

    std::sort(pv_list_.begin(), pv_list_.end(), [](const pointWithCov &x, const pointWithCov &y) {
        return x.cov.diagonal().norm() < y.cov.diagonal().norm();
    });

    updateVoxelMapOMP(pv_list_, config_.resolution, config_.max_layer, config_.layer_size,
                      config_.max_point_size, config_.max_point_size, config_.plane_thres,
                      voxel_map_);

    if (config_.verbose) {
        std::cout << "Map is updated. current map size: " << voxel_map_.size() << std::endl;
    }
}

}  // namespace se3_lio