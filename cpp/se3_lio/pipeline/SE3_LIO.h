#pragma once

// C++ standard libraries
#include <Eigen/Core>

#include <memory>

// Common
#include "common/data_type.h"
#include "common/utils.h"

// Core
#include "core/map_management.h"
#include "core/state_predict.h"
#include "core/state_update.h"

namespace se3_lio::pipeline {

struct SE3_LIO_Config {
    double acc_noise = 0.1;
    double gyr_noise = 0.1;
    double bg_noise = 0.0001;
    double ba_noise = 0.0001;

    double lidar_range_noise = 0.02;
    double lidar_angle_noise = 0.15;

    double downsample_resolution = 0.5;

    int max_iter = 4;

    double voxel_map_resolution = 1.0;
    int voxel_map_max_layer = 2;
    std::vector<int> voxel_map_layer_size = {5, 5, 5, 5, 5};
    int voxel_map_max_point_size = 1000;
    float voxel_map_plane_thres = 0.01;

    bool verbose = false;
};

class SE3_LIO {
public:
    explicit SE3_LIO(const SE3_LIO_Config &_config);
    SE3_LIO() : config_(){};

    void estimatePose(MeasurementPtr &_measurement_ptr);
    void estimatePoseWithGPS_v2(MeasurementPtr &_measurement_ptr);

    State getState() const { return state_; }

private:
    SE3_LIO_Config config_;

    MeasurementPtr meas_;
    State state_;

    se3_lio::StatePredict state_predictor_;
    se3_lio::StateUpdate state_updater_;
    std::shared_ptr<se3_lio::ManageMap> map_manager_;
};

}  // namespace se3_lio::pipeline