#pragma once

#include <random>

#include <Eigen/Dense>

#include "drake/common/constants.h"
#include "drake/common/eigen_types.h"
#include "drake/common/random.h"
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
template <typename T = double, class Generator = RandomGenerator>
Eigen::AngleAxis<T> UniformlyRandomAngleAxis(Generator* generator) {
  DRAKE_DEMAND(generator != nullptr);
  std::normal_distribution<T> normal;
  std::uniform_real_distribution<T> uniform(-M_PI, M_PI);
  const T angle = uniform(*generator);
  const T x = normal(*generator);
  const T y = normal(*generator);
  const T z = normal(*generator);
  Vector3<T> axis = Vector3<T>(x, y, z);
  if (std::is_same<T, symbolic::Expression>::value) {
    // Eigen's implementation of normalize has a conditional we skip here.
    axis /= axis.norm();
  } else {
    axis.normalize();
  }
  return Eigen::AngleAxis<T>(angle, axis);
}

/// Generates a rotation (in the quaternion representation) that rotates a
/// point on the unit sphere to another point on the unit sphere with a uniform
/// distribution over the sphere.
template <typename T = double, class Generator = RandomGenerator>
Eigen::Quaternion<T> UniformlyRandomQuaternion(Generator* generator) {
  DRAKE_DEMAND(generator != nullptr);
  const Eigen::AngleAxis<T> angle_axis = UniformlyRandomAngleAxis<T>(generator);
  return Eigen::Quaternion<T>(angle_axis);
}

/// Generates a rotation (in the rotation matrix representation) that rotates a
/// point on the unit sphere to another point on the unit sphere with a uniform
/// distribution over the sphere.
template <typename T = double, class Generator = RandomGenerator>
RotationMatrix<T> UniformlyRandomRotationMatrix(Generator* generator) {
  DRAKE_DEMAND(generator != nullptr);
  const Eigen::AngleAxis<T> angle_axis = UniformlyRandomAngleAxis<T>(generator);
  return RotationMatrix<T>(angle_axis);
}

/// Generates a rotation (in the roll-pitch-yaw representation) that rotates a
/// point on the unit sphere to another point on the unit sphere with a uniform
/// distribution over the sphere.
template <typename T = double, class Generator = RandomGenerator>
Vector3<T> UniformlyRandomRPY(Generator* generator) {
  DRAKE_DEMAND(generator != nullptr);
  const Eigen::Quaternion<T> q = UniformlyRandomQuaternion<T>(generator);
  const RollPitchYaw<T> rpy(q);
  return rpy.vector();
}

}  // namespace math
}  // namespace drake
