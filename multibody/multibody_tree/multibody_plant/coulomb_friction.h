#pragma once

#include "drake/common/drake_bool.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

/// Parameters for Coulomb's Law of Friction, namely:
/// - Static friction coefficient, for a pair of surfaces at rest relative to
///   each other.
/// - Dynamic (or kinematic) friction coefficient, for a pair of surfaces in
///   relative motion.
/// The coefficients of friction are an empirical property of the pair of
/// contacting surfaces which depend upon the mechanical properties of the
/// surfaces' materials and on the roughness of the surfaces. Friction
/// coefficients are determined experimentally.
///
/// Even though the Coulomb's law coefficients of friction characterize a pair
/// of surfaces interacting by friction, we associate the abstract __idea__ of
/// friction coefficients to a single material by considering the coefficients
/// for contact between two identical surfaces. For this case of two identical
/// surfaces, the friction coefficients that describe the surface pair are taken
/// to equal those of one of the identical surfaces. We extend this idea to the
/// case of different surfaces by defining a __combination law__ that allow us
/// to obtain the Coulomb's law coefficients of friction characterizing the pair
/// of surfaces, given the individual friction coefficients of each surface.
/// We would like this __combination law__ to satisfy:
/// - The friction coefficient of two identical surfaces is the friction
///   coefficient of one of the surfaces.
/// - The combination law is commutative. That is, surface A combined with
///   surface B gives the same results as surface B combined with surface A.
/// - For two surfaces M and N with very different friction coefficients, say
///   `μₘ ≪ μₙ`, the combined friction coefficient should be in the order of
///   magnitude of the smallest friction coefficient (in the example μₘ). To
///   understand this requirement, consider rubber (high friction coefficient)
///   sliding on ice (low friction coefficient). We'd like the surface pair
///   to be defined by a friction coefficient close to that of ice, since rubber
///   will easily slide on ice.
/// These requirements are met by the following ad-hoc combination law:
/// <pre>
///   μ = 2μₘμₙ/(μₘ + μₙ)
/// </pre>
/// See CombineWithOtherFrictionCoefficients(), which implements this law.
/// More complex combination laws could also be a function of other parameters
/// such as the mechanical properties of the interacting surfaces or even their
/// roughnesses. For instance, if the the rubber surface above has metal studs
/// (somehow making the surface "rougher"), it will definitely have a better
/// grip on an ice surface. Therefore this new variable should be taken into
/// account in the combination law.
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
  /// negative or if `dynamic_friction > static_friction` (they can be equal.)
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

  Bool<T> operator==(const CoulombFriction& other) const;

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
