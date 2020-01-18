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

/// Generates a rotation (in the quaternion representation) that rotates a
/// point on the unit sphere to another point on the unit sphere with a uniform
/// distribution over the sphere.
/// This method is briefly explained in
/// http://planning.cs.uiuc.edu/node198.html, a full explanation can be found in
/// K. Shoemake. Uniform Random Rotations in D. Kirk, editor, Graphics Gems III,
/// pages 124-132. Academic, New York, 1992.
template <typename T = double, class Generator = RandomGenerator>
Eigen::Quaternion<T> UniformlyRandomQuaternion(Generator* generator) {
  DRAKE_DEMAND(generator != nullptr);
  std::uniform_real_distribution<T> uniform(0., 1.);
  const T u1 = uniform(*generator);
  const T u2 = uniform(*generator);
  const T u3 = uniform(*generator);
  using std::sqrt;
  const T sqrt_one_minus_u1 = sqrt(1. - u1);
  const T sqrt_u1 = sqrt(u1);
  using std::cos;
  using std::sin;
  return Eigen::Quaternion<T>(sqrt_one_minus_u1 * sin(2 * M_PI * u2),
                              sqrt_one_minus_u1 * cos(2 * M_PI * u2),
                              sqrt_u1 * sin(2 * M_PI * u3),
                              sqrt_u1 * cos(2 * M_PI * u3));
}

/// Generates a rotation (in the axis-angle representation) that rotates a
/// point on the unit sphere to another point on the unit sphere with a uniform
/// distribution over the sphere.
template <typename T = double, class Generator = RandomGenerator>
Eigen::AngleAxis<T> UniformlyRandomAngleAxis(Generator* generator) {
  DRAKE_DEMAND(generator != nullptr);
  const Eigen::Quaternion<T> quaternion =
      UniformlyRandomQuaternion<T>(generator);
  return internal::QuaternionToAngleAxisLikeEigen(quaternion);
}

/// Generates a rotation (in the rotation matrix representation) that rotates a
/// point on the unit sphere to another point on the unit sphere with a uniform
/// distribution over the sphere.
template <typename T = double, class Generator = RandomGenerator>
RotationMatrix<T> UniformlyRandomRotationMatrix(Generator* generator) {
  DRAKE_DEMAND(generator != nullptr);
  const Eigen::Quaternion<T> quaternion =
      UniformlyRandomQuaternion<T>(generator);
  return RotationMatrix<T>(quaternion);
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
