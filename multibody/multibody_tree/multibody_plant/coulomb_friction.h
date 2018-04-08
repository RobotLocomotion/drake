#pragma once

#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

/// Parameters for the Coulomb's Law of Friction, namely:
/// - Static friction coefficient, for surfaces at rest.
/// - Dynamic (or kinematic) friction coefficient, for surfaces in relative
///   motion.
/// The coefficients of friction are an empirical property of the contacting
/// surfaces which depend upon the mechanical properties of the surfaces's
/// materials and on the roughness of the surfaces. Friction coefficients are
/// determined experimentally.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
/// - symbolic::Expression
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template<typename T>
class CoulombFriction {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CoulombFriction)

  /// Default constructor for a frictionless surface, i.e. with zero static and
  /// dynamic coefficients of friction.
  CoulombFriction() = default;

  /// Specifies both the static and dynamic friction coefficients for a given
  /// surface.
  /// @throws std::runtime_error if any of the friction coefficients are
  /// negative or if the dynamic friction coefficient is strictly higher than
  /// the static friction coefficient (they can be equal.)
  CoulombFriction(const T& static_friction, const T& dynamic_friction);

  /// Combines `this` friction coefficients for a surface M with `other` set of
  /// friction coefficients for a surface N according to: <pre>
  ///   μ = 2μₘμₙ/(μₘ + μₙ)
  /// </pre>
  /// where the operation above is performed separately on the static and
  /// dynamic friction coefficients.
  /// @returns the combined friction coefficients for the interacting surfaces.
  CoulombFriction CombineWithOtherFrictionCoefficients(
      const CoulombFriction& other) const {
    // Simple utility to detect 0 / 0. As it is used in this method, denom
    // can only be zero if num is also zero, so we'll simply return zero.
    auto safe_divide = [](const T& num, const T& denom) {
      return denom == 0.0 ? 0.0 : num / denom;
    };
    return CoulombFriction(
        safe_divide(
            2 * static_friction() * other.static_friction(),
            static_friction() + other.static_friction()),
        safe_divide(
            2 * dynamic_friction() * other.dynamic_friction(),
            dynamic_friction() + other.dynamic_friction()));
  }

  /// Returns the coefficient of static friction.
  const T& static_friction() const { return static_friction_; }

  /// Returns the coefficient of dynamic friction.
  const T& dynamic_friction() const { return dynamic_friction_; }

 private:
  // Confirms two properties on the friction coefficient pair:
  //  1. Both values non-negative.
  //  2. static_friction >= dynamic_friction.
  // Throws std::runtime_error on failure of these tests.
  static void ThrowForBadFriction(const T& static_friction,
                                  const T& dynamic_friction);

  // Default values are for an ideal frictionless material.
  T static_friction_{0.0};
  T dynamic_friction_{0.0};
};

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
