#pragma once

#include <random>

#include <Eigen/Dense>

#include "drake/common/constants.h"
#include "drake/common/eigen_types.h"
#include "drake/common/drake_export.h"

namespace drake {
namespace math {
DRAKE_EXPORT Eigen::Vector4d UniformlyRandomAxisAngle(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    std::default_random_engine& generator);
DRAKE_EXPORT Eigen::Vector4d UniformlyRandomQuat(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    std::default_random_engine& generator);
DRAKE_EXPORT Eigen::Matrix3d UniformlyRandomRotmat(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    std::default_random_engine& generator);
DRAKE_EXPORT Eigen::Vector3d UniformlyRandomRPY(
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    std::default_random_engine& generator);
}  // namespace math
}  // namespace drake
