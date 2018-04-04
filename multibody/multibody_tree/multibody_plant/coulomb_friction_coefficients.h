#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

/** The set of per-object compliant material parameters with one material
 applied to each collision object. The material parameters include:
   - Young's modulus with units of pascals (i.e.,N/m²). This is a measure of the
     tensile elasticity of the material and, as such, may also be referred to
     as "elasticity" or, occasionally, "stiffness". The default value is
     that of a hard rubber: 1e8 pascals.
   - dissipation with units of s/m (i.e., 1/velocity). Its default value is
     0.32, drawn from the Hunt-Crossly 1975 paper representing the dissipation
     for ivory.
   - coefficients of friction (static and dynamic). Unitless values with
     default values of 0.9 for the static coefficient of friction and 0.5 for
     the dynamic coefficient.

 Each collision geometry is associated with compliant contact material
 properties. The value of properties can be _explicit_ or _default_.

 Explicit values are those values that are explicitly set (via the API or in a
 source specification file -- e.g., URDF or SDF). When queried, the property set
 with explicit values will always report the explicit value.

 Default values are left "open". For default values the context in which the
 property value is queried matters. Material properties set to be "default" will
 return the hard-coded default value or a user-provided default value. Thus,
 two different invocations on the _same_ instance can provide different values
 iff the property is configured to be default, and the different invocations
 provide different default values.

 Using the dissipation property to provide a concrete example:

 ```
 // Constructor sets all properties to be default configured.
 CoulombFrictionCoefficients material;

 material.dissipation();        // Reports the hard-coded default value.
 material.dissipation(1.2);     // Reports the provided default value, 1.2.

 material.set_dissipation(10);  // dissipation is no longer default.

 material.dissipation();        // Reports the explicit value, 10.
 material.dissipation(1.2);     // Reports the explicit value, 10.
 ```

 See @ref drake_contacts for semantics of these properties for dynamics. */
// TODO(SeanCurtis-TRI): Investigate templatizing this on scalar.
class CoulombFrictionCoefficients {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CoulombFrictionCoefficients)
  CoulombFrictionCoefficients() = default;

  /** Constructs fully specified material. Will throw an exception in any of the
   following circumstances:
     - `youngs_modulus` <= 0
     - `dissipation` < 0
     - `static_friction` < 0
     - `dynamic_friction` < 0
     - `static_friction` < `dynamic_friction`

   No value will be configured to use default values. */
  CoulombFrictionCoefficients(double static_friction, double dynamic_friction);

  CoulombFrictionCoefficients CombineWithOtherFrictionCoefficients(
      const CoulombFrictionCoefficients& other) const {
    // Simple utility to detect 0 / 0. As it is used in this method, denom
    // can only be zero if num is also zero, so we'll simply return zero.
    auto safe_divide = [](double num, double denom) {
      return denom == 0.0 ? 0.0 : num / denom;
    };
    return CoulombFrictionCoefficients(
        safe_divide(
            2 * static_friction() * other.static_friction(),
            static_friction() + other.static_friction()),
        safe_divide(
            2 * dynamic_friction() * other.dynamic_friction(),
            dynamic_friction() + other.dynamic_friction()));
  }

  double static_friction() const { return static_friction_; }

  double dynamic_friction() const { return dynamic_friction_; }

 private:
  // Confirms two properties on the friction coefficient pair:
  //  1. Both values non-negative.
  //  2. static_friction >= dynamic_friction.
  // Throws std::runtime_error on failure of these tests.
  static void ThrowForBadFriction(double static_friction,
                                  double dynamic_friction);

  // Default values are for an ideal frictionless material.
  double static_friction_{0.0};
  double dynamic_friction_{0.0};
};

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
