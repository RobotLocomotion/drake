#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <random>

#include "drake/common/constants.h"
#include "drake/common/eigen_types.h"
#include "drake/drakeMath_export.h"
#include "drake/math/quaternion.h"

namespace drake {
namespace math {
DRAKEMATH_EXPORT Eigen::Vector4d uniformlyRandomAxisAngle(
    std::default_random_engine& generator);
DRAKEMATH_EXPORT Eigen::Vector4d uniformlyRandomQuat(
    std::default_random_engine& generator);
DRAKEMATH_EXPORT Eigen::Matrix3d uniformlyRandomRotmat(
    std::default_random_engine& generator);
DRAKEMATH_EXPORT Eigen::Vector3d uniformlyRandomRPY(
    std::default_random_engine& generator);
}  // namespace math
}  // namespace drake
