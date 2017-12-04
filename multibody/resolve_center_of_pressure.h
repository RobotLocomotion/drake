#pragma once

#include <utility>

#include <Eigen/Dense>

// TODO(siyuan.feng): Cleanup the naming according to the style guide.

template <typename DerivedTorque, typename DerivedForce, typename DerivedNormal,
          typename DerivedPoint>
std::pair<Eigen::Vector3d, double> resolveCenterOfPressure(
    const Eigen::MatrixBase<DerivedTorque>& torque,
    const Eigen::MatrixBase<DerivedForce>& force,
    const Eigen::MatrixBase<DerivedNormal>& normal,
    const Eigen::MatrixBase<DerivedPoint>& point_on_contact_plane);
