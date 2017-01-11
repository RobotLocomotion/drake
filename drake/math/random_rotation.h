#pragma once

#include <random>

#include <Eigen/Dense>

#include "drake/common/constants.h"
#include "drake/common/eigen_types.h"
#include "drake/math/axis_angle.h"

namespace drake {
namespace math {

template< class Generator >
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
Eigen::Vector4d UniformlyRandomAxisAngle(Generator& generator) {
  std::normal_distribution<double> normal;
  std::uniform_real_distribution<double> uniform(-M_PI, M_PI);
  double angle = uniform(generator);
  Eigen::Vector3d axis =
      Eigen::Vector3d(normal(generator), normal(generator), normal(generator));
  axis.normalize();
  Eigen::Vector4d a;
  a << axis, angle;
  return a;
}

template< class Generator >
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
Eigen::Vector4d UniformlyRandomQuat(Generator& generator) {
  return axis2quat(UniformlyRandomAxisAngle(generator));
}

template< class Generator >
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
Eigen::Matrix3d UniformlyRandomRotmat(Generator& generator) {
  return axis2rotmat(UniformlyRandomAxisAngle(generator));
}

template< class Generator >
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
Eigen::Vector3d UniformlyRandomRPY(Generator& generator) {
  return axis2rpy(UniformlyRandomAxisAngle(generator));
}

}  // namespace math
}  // namespace drake
