#pragma once

#include <random>

#include <Eigen/Dense>

#include "drake/common/constants.h"
#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"

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
Eigen::AngleAxisd UniformlyRandomAngleAxis(Generator& generator) {
  std::normal_distribution<double> normal;
  std::uniform_real_distribution<double> uniform(-M_PI, M_PI);
  const double angle = uniform(generator);
  Eigen::Vector3d axis(normal(generator), normal(generator), normal(generator));
  axis.normalize();
  return Eigen::AngleAxisd(angle, axis);
}

/// Generates a rotation (in the quaternion representation) that rotates a
/// point on the unit sphere to another point on the unit sphere with a uniform
/// distribution over the sphere.
// TODO(mitiguy) change this method so it returns an Eigen::Quaternion.
template <class Generator>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
Eigen::Vector4d UniformlyRandomQuat(Generator& generator) {
  const Eigen::AngleAxisd angle_axis = UniformlyRandomAngleAxis(generator);
  const Eigen::Quaterniond q(angle_axis);
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

/// Generates a rotation (in the rotation matrix representation) that rotates a
/// point on the unit sphere to another point on the unit sphere with a uniform
/// distribution over the sphere.
template <class Generator>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
RotationMatrix<double> UniformlyRandomRotationMatrix(Generator& generator) {
  const Eigen::AngleAxisd angle_axis = UniformlyRandomAngleAxis(generator);
  return RotationMatrix<double>(angle_axis);
}

/// Generates a rotation (in the roll-pitch-yaw representation) that rotates a
/// point on the unit sphere to another point on the unit sphere with a uniform
/// distribution over the sphere.
template <class Generator>
// TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
Eigen::Vector3d UniformlyRandomRPY(Generator& generator) {
  // TODO(mitiguy) Replace with single call to UniformlyRandomQuaternion when
  // that function exists and returns Eigen::Quaterniond (rather than Vector4).
  const Eigen::AngleAxisd angle_axis = UniformlyRandomAngleAxis(generator);
  const Eigen::Quaterniond q(angle_axis);
  return QuaternionToSpaceXYZ(q);
}

}  // namespace math
}  // namespace drake
