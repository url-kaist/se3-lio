#ifndef LIE_H
#define LIE_H

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include "common/data_type.h"

namespace se3_lio {

/**
 * @brief propagate state with dx
 *
 * @param state current state
 * @param dx perturbation represented in tangent space of current state
 * @return propagated state
 */
State boxplus(const State &state, const ErrVector &delta);

/**
 * @brief compute dx such that [tgt_state = boxplus(src_state, dx)]
 *
 * @param tgt_state target state
 * @param src_state source state
 * @return perturbation represented in tangent space of source state
 */
ErrVector boxminus(const State &tgt_state, const State &src_state);

/**
 * @brief reset current error state to tangent space of target state, [src = dx
 * + J*tgt]
 *
 * @param src_state source state
 * @param tgt_state target state
 * @return (dx, J) difference and Jacobian between target and source state
 * state
 */
std::tuple<ErrVector, CovMatrix> resetErrorState(const State &src_state, const State &tgt_state);

inline Eigen::Matrix3d skew_sym(const Eigen::Vector3d &v) {
    Eigen::Matrix3d skew;
    skew << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return skew;
};

inline Eigen::Matrix3d calculateMeasurementNoise(const Eigen::Vector3d &_point,
                                                 double _ranging_sig,
                                                 double _angle_sig) {
    double range_var = _ranging_sig * _ranging_sig;
    Eigen::Matrix2d angle_var;
    angle_var << std::pow(std::sin(_angle_sig * M_PI / 180), 2), 0, 0,
        std::pow(std::sin(_angle_sig * M_PI / 180), 2);

    double range = _point.norm();

    Eigen::Vector3d dir = _point.normalized();
    Eigen::Matrix3d dir_skew = skew_sym(dir);

    Eigen::Vector3d base_vector1(1, 1, -(dir(0) + dir(1)) / dir(2));
    base_vector1.normalize();

    Eigen::Vector3d base_vector2 = base_vector1.cross(dir);
    base_vector2.normalize();

    Eigen::Matrix<double, 3, 2> N;
    N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2),
        base_vector2(2);

    Eigen::Matrix<double, 3, 2> A = range * dir_skew * N;

    Eigen::Matrix3d noise = dir * range_var * dir.transpose() + A * angle_var * A.transpose();

    return noise;
};

}  // namespace se3_lio

#endif  // LIE_H
