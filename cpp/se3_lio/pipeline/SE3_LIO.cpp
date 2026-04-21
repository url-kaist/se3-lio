#include "SE3_LIO.h"

namespace se3_lio::pipeline {

SE3_LIO::SE3_LIO(const SE3_LIO_Config &_config) : config_(_config) {
    state_predictor_.setIMUNoise(config_.gyr_noise, config_.acc_noise, config_.bg_noise,
                                 config_.ba_noise);
    state_predictor_.setLiDARNoise(config_.lidar_range_noise, config_.lidar_angle_noise);

    se3_lio::ManageMapConfig manage_map_config;
    manage_map_config.resolution = config_.voxel_map_resolution;
    manage_map_config.max_layer = config_.voxel_map_max_layer;
    manage_map_config.layer_size = config_.voxel_map_layer_size;
    manage_map_config.max_point_size = config_.voxel_map_max_point_size;
    manage_map_config.plane_thres = config_.voxel_map_plane_thres;

    map_manager_ = std::make_shared<se3_lio::ManageMap>(manage_map_config);

    state_updater_ = se3_lio::StateUpdate(config_.max_iter);
    state_updater_.setManageMapConfig(manage_map_config);
    state_updater_.setMap(map_manager_);
}

void SE3_LIO::estimatePose(MeasurementPtr &_measurement_ptr) {
    state_predictor_.setState(state_);
    state_predictor_.setMeasurement(_measurement_ptr);
    state_ = state_predictor_.predictState();
    state_predictor_.undistortCloud();

    if (!state_predictor_.isValid()) return;

    _measurement_ptr->raw_lidar = _measurement_ptr->lidar;
    downsampleCloud(_measurement_ptr->lidar, config_.downsample_resolution);

    state_predictor_.calculateUndistCloudCov(_measurement_ptr->lidar);

    state_updater_.setState(state_);
    state_updater_.setMeasurement(_measurement_ptr);
    state_ = state_updater_.updateState();

    map_manager_->setState(state_);
    map_manager_->setMeasurement(_measurement_ptr->lidar);
    map_manager_->updateMap();
}

void SE3_LIO::estimatePoseWithGPS_v2(MeasurementPtr &_measurement_ptr) {
    state_predictor_.setState(state_);
    state_predictor_.setMeasurement(_measurement_ptr);
    state_ = state_predictor_.predictState();
    state_predictor_.undistortCloud();

    if (!state_predictor_.isValid()) return;

    downsampleCloud(_measurement_ptr->lidar, config_.downsample_resolution);

    state_predictor_.calculateUndistCloudCov(_measurement_ptr->lidar);

    state_updater_.setState(state_);
    state_updater_.setMeasurement(_measurement_ptr);
    state_ = state_updater_.updateState_v2();

    map_manager_->setState(state_);
    map_manager_->setMeasurement(_measurement_ptr->lidar);
    map_manager_->updateMap();
}

}  // namespace se3_lio::pipeline