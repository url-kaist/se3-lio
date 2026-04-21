#include "lie.h"

namespace se3_lio {

State boxplus(const State &state, const ErrVector &delta) {
    State new_state = state;
    new_state.stamp = state.stamp;
    new_state.pose = state.pose * Sophus::SE3d::exp(delta.block<6, 1>(POSE, 0)).matrix();
    new_state.vel = Sophus::SO3d::exp(delta.block<3, 1>(ROT, 0)).inverse() *
                    (state.vel + delta.block<3, 1>(VEL, 0));
    new_state.bg = state.bg + delta.block<3, 1>(B_G, 0);
    new_state.ba = state.ba + delta.block<3, 1>(B_A, 0);
    new_state.grav = (state.grav + delta.block<3, 1>(GRAV, 0)).normalized() * 9.81;

    return new_state;
}

ErrVector boxminus(const State &tgt_state, const State &src_state) {
    ErrVector dx = ErrVector::Zero();

    dx.block<6, 1>(POSE, 0) = Sophus::SE3d(src_state.pose.inverse() * tgt_state.pose).log();
    dx.block<3, 1>(VEL, 0) =
        Sophus::SO3d::exp(dx.block<3, 1>(ROT, 0)) * tgt_state.vel - src_state.vel;
    dx.block<3, 1>(B_G, 0) = tgt_state.bg - src_state.bg;
    dx.block<3, 1>(B_A, 0) = tgt_state.ba - src_state.ba;
    dx.block<3, 1>(GRAV, 0) = tgt_state.grav - src_state.grav;

    return dx;
}

std::tuple<ErrVector, CovMatrix> resetErrorState(const State &src_state, const State &tgt_state) {
    ErrVector dx_prior = boxminus(tgt_state, src_state);
    CovMatrix J_prior = CovMatrix::Identity();

    J_prior.block<6, 6>(POSE, POSE) =
        Sophus::SE3d::leftJacobian(-dx_prior.block<6, 1>(POSE, 0)).matrix().inverse();
    J_prior.block<3, 3>(VEL, VEL) = Sophus::SO3d::exp(dx_prior.block<3, 1>(ROT, 0)).matrix();

    return std::make_tuple(dx_prior, J_prior);
}

}  // namespace se3_lio