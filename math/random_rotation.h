#pragma once

#include <random>

#include <Eigen/Dense>

#include "drake/common/constants.h"
#include "drake/common/eigen_types.h"
#include "drake/math/quaternion.h"
#include "drake/math/roll_pitch_yaw.h"

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
Eigen::AngleAxisd UniformlyRandomAngleAxis(Generator* generator) {
  DRAKE_DEMAND(generator != nullptr);
  std::normal_distribution<double> normal;
  std::uniform_real_distribution<double> uniform(-M_PI, M_PI);
  const double angle = uniform(*generator);
  const double x = normal(*generator);
  const double y = normal(*generator);
  const double z = normal(*generator);
  const Eigen::Vector3d axis = Eigen::Vector3d(x, y, z).normalized();
  return Eigen::AngleAxisd(angle, axis);
}

/// Generates a rotation (in the quaternion representation) that rotates a
/// point on the unit sphere to another point on the unit sphere with a uniform
/// distribution over the sphere.
template <class Generator>
Eigen::Quaterniond UniformlyRandomQuaternion(Generator* generator) {
  DRAKE_DEMAND(generator != nullptr);
  const Eigen::AngleAxisd angle_axis = UniformlyRandomAngleAxis(generator);
  return Eigen::Quaterniond(angle_axis);
}

/// Generates a rotation (in the rotation matrix representation) that rotates a
/// point on the unit sphere to another point on the unit sphere with a uniform
/// distribution over the sphere.
template <class Generator>
RotationMatrix<double> UniformlyRandomRotationMatrix(Generator* generator) {
  DRAKE_DEMAND(generator != nullptr);
  const Eigen::AngleAxisd angle_axis = UniformlyRandomAngleAxis(generator);
  return RotationMatrix<double>(angle_axis);
}

/// Generates a rotation (in the roll-pitch-yaw representation) that rotates a
/// point on the unit sphere to another point on the unit sphere with a uniform
/// distribution over the sphere.
template <class Generator>
Eigen::Vector3d UniformlyRandomRPY(Generator* generator) {
  DRAKE_DEMAND(generator != nullptr);
  const Eigen::Quaterniond q = UniformlyRandomQuaternion(generator);
  const RollPitchYaw<double> rpy(q);
  return rpy.vector();
}

}  // namespace math
}  // namespace drake
