#include "drake/math/random_rotation.h"

#include "drake/math/axis_angle.h"

using Eigen::Vector3d;
using Eigen::Vector4d;

namespace drake {
namespace math {
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
Vector4d UniformlyRandomAxisAngle(std::default_random_engine& generator) {
  std::normal_distribution<double> normal;
  std::uniform_real_distribution<double> uniform(-M_PI, M_PI);
  double angle = uniform(generator);
  Vector3d axis =
      Vector3d(normal(generator), normal(generator), normal(generator));
  axis.normalize();
  Vector4d a;
  a << axis, angle;
  return a;
}

// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
Vector4d UniformlyRandomQuat(std::default_random_engine& generator) {
  return drake::math::axis2quat(UniformlyRandomAxisAngle(generator));
}

// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
Eigen::Matrix3d UniformlyRandomRotmat(std::default_random_engine& generator) {
  return drake::math::axis2rotmat(UniformlyRandomAxisAngle(generator));
}

// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
Eigen::Vector3d UniformlyRandomRPY(std::default_random_engine& generator) {
  return drake::math::axis2rpy(UniformlyRandomAxisAngle(generator));
}
}  // namespace math
}  // namespace drake
