
#ifndef MANAGE_MAP_H
#define MANAGE_MAP_H

// C++ standard libraries
// #include <boost/filesystem.hpp>
#include <filesystem>
#include <mutex>
#include <queue>

// Eigen library
#include <Eigen/Dense>

// Common
#include "common/data_type.h"
#include "common/utils.h"

// core
#include "core/lie.h"
#include "core/temporary/voxel_map_util.h"

namespace se3_lio {

struct ManageMapConfig {
    double resolution = 1.0;
    int max_layer = 2;
    std::vector<int> layer_size = {5, 5, 5, 5, 5};
    // int max_point_size = 1000;
    int max_point_size;
    float plane_thres = 0.01f;
    bool verbose = false;
};

class ManageMap {
public:
    explicit ManageMap(ManageMapConfig _config);
    ManageMap() : config_() { is_initialized_ = false; }

    ~ManageMap(){};

    /**
     * @brief Set the State object
     *
     * @param _state
     */
    void setState(State _state);

    /**
     * @brief Set the Synced Measurement object
     *
     * @param _measurement
     */
    void setMeasurement(LiDAR _measurement);

    /**
     * @brief Initialize the map
     *
     */
    void initMap();

    /**
     * @brief Update the map
     *
     */
    void updateMap();

    /**
     * @brief Check whether the map is initialized
     *
     * @return true or false
     */
    bool isInitialized() { return is_initialized_; }

    /**
     * @brief Get the planes
     *
     */
    std::vector<Plane> getPlanes();

    std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map_;

private:
    ManageMapConfig config_;

    State curr_state_;
    LiDAR undistorted_cloud_;

    Eigen::Matrix4d curr_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 6, 6> curr_pose_cov_;

    std::vector<pointWithCov> pv_list_;

    bool is_initialized_;
};

}  // namespace se3_lio

#endif  // MANAGE_MAP_H