#ifndef SE3_LIO_COMMON_UTILS_H
#define SE3_LIO_COMMON_UTILS_H

#include <Eigen/Dense>
#include <unordered_map>

#include "data_type.h"

template <>
struct std::hash<Eigen::Vector3i> {
    std::size_t operator()(const Eigen::Vector3i &voxel) const {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        return (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
    }
};

namespace se3_lio {

inline PointCloudType transformPointCloud(
    const PointCloudType &_cloud, const Eigen::Matrix4d &_transform) {
    PointCloudType transformed_cloud;
    transformed_cloud.reserve(_cloud.size());

    Eigen::Matrix4f tsfm_float = _transform.cast<float>();
    for (const auto &point : _cloud) {
        CustomPointType transformed_point;
        Eigen::Vector4f point_homo = point.getVector4fMap();
        point_homo[3] = 1.0f;
        Eigen::Vector4f transformed_point_homo = tsfm_float * point_homo;
        transformed_point.x = transformed_point_homo(0);
        transformed_point.y = transformed_point_homo(1);
        transformed_point.z = transformed_point_homo(2);
        transformed_point.intensity = point.intensity;
        transformed_point.timestamp = point.timestamp;
        transformed_cloud.push_back(transformed_point);
    }

    return transformed_cloud;
};

inline void downsampleCloud(LiDAR &_cloud, const double _resolution) {
    std::unordered_map<Eigen::Vector3i, std::pair<CustomPointType, double>> grid;
    grid.reserve(_cloud.points.size());

    for (const auto &point : _cloud.points) {
        Eigen::Vector3i key(static_cast<int>(std::floor(point.x / _resolution)),
                            static_cast<int>(std::floor(point.y / _resolution)),
                            static_cast<int>(std::floor(point.z / _resolution)));

        float cx = (key.x() + 0.5f) * _resolution;
        float cy = (key.y() + 0.5f) * _resolution;
        float cz = (key.z() + 0.5f) * _resolution;

        double dist2 = (point.x - cx) * (point.x - cx) + (point.y - cy) * (point.y - cy) +
                       (point.z - cz) * (point.z - cz);

        auto it = grid.find(key);
        if (it == grid.end()) {
            grid[key] = {point, dist2};
        } else if (dist2 < it->second.second) {
            grid[key] = {point, dist2};
        }
    }

    PointCloudType downsampled_cloud_out;
    downsampled_cloud_out.reserve(grid.size());

    for (const auto &voxel_and_point : grid) {
        downsampled_cloud_out.push_back(voxel_and_point.second.first);
    }

    _cloud.points = downsampled_cloud_out;
};

}  // namespace se3_lio

#endif  // SE3_LIO_COMMON_UTILS_H