#pragma once

#include <random>

#include <Eigen/Dense>

#include "drake/common/constants.h"
#include "drake/common/eigen_types.h"
#include "drake/math/axis_angle.h"

namespace drake {
namespace math {

/// Generates a rotation (in the axis-angle representation) that rotates a
/// point on the unit sphere to another point on the unit sphere with a uniform
/// distribution over the sphere.
///
/// Justification for the algorithm can be found in, e.g.:
/// Mervin E. Muller. 1959. A note on a method for generating points
/// uniformly on n-dimensional spheres. Commun. ACM 2, 4 (April 1959), 19-20.
/// DOI=http://dx.doi.org/10.1145/377939.377946
template <class Generator>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
Eigen::Vector4d UniformlyRandomAxisAngle(Generator& generator) {
  std::normal_distribution<double> normal;
  std::uniform_real_distribution<double> uniform(-M_PI, M_PI);
  double angle = uniform(generator);
  Eigen::Vector3d axis(normal(generator), normal(generator), normal(generator));
  axis.normalize();
  Eigen::Vector4d a;
  a << axis, angle;
  return a;
}

/// Generates a rotation (in the quaternion representation) that rotates a
/// point on the unit sphere to another point on the unit sphere with a uniform
/// distribution over the sphere.
template <class Generator>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
Eigen::Vector4d UniformlyRandomQuat(Generator& generator) {
  return axis2quat(UniformlyRandomAxisAngle(generator));
}

/// Generates a rotation (in the rotation matrix representation) that rotates a
/// point on the unit sphere to another point on the unit sphere with a uniform
/// distribution over the sphere.
template <class Generator>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
Eigen::Matrix3d UniformlyRandomRotmat(Generator& generator) {
  return axis2rotmat(UniformlyRandomAxisAngle(generator));
}

/// Generates a rotation (in the roll-pitch-yaw representation) that rotates a
/// point on the unit sphere to another point on the unit sphere with a uniform
/// distribution over the sphere.
template <class Generator>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
Eigen::Vector3d UniformlyRandomRPY(Generator& generator) {
  return axis2rpy(UniformlyRandomAxisAngle(generator));
}

}  // namespace math
}  // namespace drake
