#pragma once

#include "drake/common/default_scalars.h"
#include "drake/common/drake_bool.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace multibody {

/// Parameters for Coulomb's Law of Friction, namely:
///
/// - Static friction coefficient, for a pair of surfaces at rest relative to
///   each other.
/// - Dynamic (or kinematic) friction coefficient, for a pair of surfaces in
///   relative motion.
///
/// These coefficients are an empirical property characterizing the
/// interaction by friction between a pair of contacting surfaces. Friction
/// coefficients depend upon the mechanical properties of the surfaces'
/// materials and on the roughness of the surfaces. They are determined
/// experimentally.
///
/// Even though the Coulomb's law coefficients of friction characterize a pair
/// of surfaces interacting by friction, we associate the abstract __idea__ of
/// friction coefficients to a single surface by considering the coefficients
/// for contact between two identical surfaces. For this case of two identical
/// surfaces, the friction coefficients that describe the surface pair are taken
/// to equal those of one of the identical surfaces. We extend this idea to the
/// case of different surfaces by defining a __combination law__ that allow us
/// to obtain the Coulomb's law coefficients of friction characterizing the pair
/// of surfaces, given the individual friction coefficients of each surface.
/// We would like this __combination law__ to satisfy:
///
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
///
/// These requirements are met by the following ad-hoc combination law:
/// <pre>
///   μ = 2μₘμₙ/(μₘ + μₙ)
/// </pre>
/// See CalcContactFrictionFromSurfaceProperties(), which implements this law.
/// More complex combination laws could also be a function of other parameters
/// such as the mechanical properties of the interacting surfaces or even their
/// roughnesses. For instance, if the rubber surface above has metal studs
/// (somehow making the surface "rougher"), it will definitely have a better
/// grip on an ice surface. Therefore this new variable should be taken into
/// account in the combination law. Notice that in this example, this new
/// combination law model for tires, will have a different set of requirements
/// from the ones stated above.
///
/// @tparam_default_scalar
template<typename T>
class CoulombFriction {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CoulombFriction)

  /// Default constructor for a frictionless surface, i.e. with zero static and
  /// dynamic coefficients of friction.
  CoulombFriction() = default;

  /// Specifies both the static and dynamic friction coefficients for a given
  /// surface.
  /// @throws std::exception if any of the friction coefficients are
  /// negative or if `dynamic_friction > static_friction` (they can be equal.)
  CoulombFriction(const T& static_friction, const T& dynamic_friction);

  /// Returns the coefficient of static friction.
  const T& static_friction() const { return static_friction_; }

  /// Returns the coefficient of dynamic friction.
  const T& dynamic_friction() const { return dynamic_friction_; }

  /// Performs a bitwise-identical comparison, not done to any tolerance.
  boolean<T> operator==(const CoulombFriction& other) const;

 private:
  // Confirms two properties on the friction coefficient pair:
  //  1. Both values non-negative.
  //  2. static_friction >= dynamic_friction.
  // Throws std::exception on failure of these tests.
  static void ThrowForBadFriction(const T& static_friction,
                                  const T& dynamic_friction);

  // Default values are for an ideal frictionless surface.
  T static_friction_{0.0};
  T dynamic_friction_{0.0};
};

/// Given the surface properties of two different surfaces, this method computes
/// the Coulomb's law coefficients of friction characterizing the interaction by
/// friction of the given surface pair.
/// The surface properties are specified by individual Coulomb's law
/// coefficients of friction. As outlined in the class's documentation for
/// CoulombFriction, friction coefficients characterize a surface pair and not
/// individual surfaces. However, we find it useful in practice to associate the
/// abstract __idea__ of friction coefficients to a single surface. Please refer
/// to the documentation for CoulombFriction for details on this topic.
///
/// More specifically, this method computes the contact coefficients for the
/// given surface pair as: <pre>
///   μ = 2μₘμₙ/(μₘ + μₙ)
/// </pre>
/// where the operation above is performed separately on the static and
/// dynamic friction coefficients.
///
/// @param[in] surface_properties1
///   Surface properties for surface 1. Specified as an individual set of
///   Coulomb's law coefficients of friction.
/// @param[in] surface_properties2
///   Surface properties for surface 2. Specified as an individual set of
///   Coulomb's law coefficients of friction.
/// @returns the combined friction coefficients for the interacting surfaces.
template<typename T>
CoulombFriction<T> CalcContactFrictionFromSurfaceProperties(
    const CoulombFriction<T>& surface_properties1,
    const CoulombFriction<T>& surface_properties2) {
  // Aliases to shorten expressions below.
  const auto& s1 = surface_properties1;
  const auto& s2 = surface_properties2;
  // Simple utility to detect 0 / 0. As it is used in this method, denom
  // can only be zero if num is also zero, so we'll simply return zero.
  auto safe_divide = [](const T& num, const T& denom) {
    return denom == 0.0 ? 0.0 : num / denom;
  };
  return CoulombFriction<T>(
      safe_divide(
          2 * s1.static_friction() * s2.static_friction(),
          s1.static_friction() + s2.static_friction()),
      safe_divide(
          2 * s1.dynamic_friction() * s2.dynamic_friction(),
          s1.dynamic_friction() + s2.dynamic_friction()));
}

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::CoulombFriction);
