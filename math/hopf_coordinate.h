/// @file
/// Hopf coodinates parameterizes SO(3) locally as the Cartesian product of a
/// one-sphere and a two-sphere S¹ x S². Computationally, each rotation in the
/// Hopf coordinates can be written as (θ, φ, ψ), in which ψ parameterizes the
/// circle S¹ and has a range of 2π, and θ, φ represent the spherical
/// coordinates for S², with the range of π and 2π respectively.

#pragma once

#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace math {
/**
 * Transforms Hopf coordinates to a quaternion w, x, y, z as
 * w = cos(θ/2)cos(ψ/2)
 * x = cos(θ/2)sin(ψ/2)
 * y = sin(θ/2)cos(φ+ψ/2)
 * z = sin(θ/2)sin(φ+ψ/2)
 * The user can refer to equation 5 of
 * Generating Uniform Incremental Grids on SO(3) Using the Hopf Fibration
 * by Anna Yershova, Steven LaValle and Julie Mitchell, 2008
 * @param theta The θ angle.
 * @param phi The φ angle.
 * @param psi The ψ angle.
 */
template <typename T>
const Eigen::Quaternion<T> HopfCoordinateToQuaternion(const T& theta,
                                                      const T& phi,
                                                      const T& psi) {
  using std::cos;
  using std::sin;
  const T cos_half_theta = cos(theta / 2);
  const T sin_half_theta = sin(theta / 2);
  const T phi_plus_half_psi = phi + psi / 2;
  return Eigen::Quaternion(cos_half_theta * cos(psi / 2),
                           cos_half_theta * sin(psi / 2),
                           sin_half_theta * cos(phi_plus_half_psi),
                           sin_half_theta * sin(phi_plus_half_psi));
}

/**
 * Convert a unit-length quaternion (w, x, y, z) to Hopf coordinate as
 * if w >= 0
 *   ψ = 2*atan2(x, w)
 * else
 *   ψ = 2*atan2(-x, -w)
 * φ = mod(atan2(z, y) - ψ/2, 2pi)
 * θ = 2*atan2(√(y²+z²), √(w²+x²))
 * ψ is in the range of [-pi, pi].
 * φ is in the range of [0, 2pi].
 * θ is in the range of [0, pi].
 * @param quaternion A unit length quaternion.
 * @return hopf_coordinate (θ, φ, ψ) as an Eigen vector.
 */
template <typename T>
Vector3<T> QuaternionToHopfCoordinate(const Eigen::Quaternion<T>& quaternion) {
  using std::atan2;
  const T psi = quaternion.w() >= T(0)
                    ? 2 * atan2(quaternion.x(), quaternion.w())
                    : 2 * atan2(-quaternion.x(), -quaternion.w());
  const T phi_unwrapped = atan2(quaternion.z(), quaternion.y()) - psi / 2;
  // The range of phi_unwrapped is [-1.5pi, 1.5pi]
  const T phi = phi_unwrapped >= 0 ? phi_unwrapped : phi_unwrapped + 2 * M_PI;
  using std::pow;
  using std::sqrt;
  const T theta =
      2 * atan2(sqrt(pow(quaternion.y(), 2) + pow(quaternion.z(), 2)),
                sqrt(pow(quaternion.w(), 2) + pow(quaternion.x(), 2)));
  return Vector3<T>(theta, phi, psi);
}
}  // namespace math.
}  // namespace drake
