#pragma once

#include <optional>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace systems {

// TODO(SeanCurtis-TRI): Investigate templatizing this on scalar.
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
 CompliantMaterial material;

 material.dissipation();        // Reports the hard-coded default value.
 material.dissipation(1.2);     // Reports the provided default value, 1.2.

 material.set_dissipation(10);  // dissipation is no longer default.

 material.dissipation();        // Reports the explicit value, 10.
 material.dissipation(1.2);     // Reports the explicit value, 10.
 ```

 See @ref drake_contacts for semantics of these properties for dynamics. */
class CompliantMaterial {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CompliantMaterial)

  /** Constructs a default material. */
  CompliantMaterial() = default;

  /** Constructs fully specified material. Will throw an exception in any of the
   following circumstances:

     - `youngs_modulus` <= 0
     - `dissipation` < 0
     - `static_friction` < 0
     - `dynamic_friction` < 0
     - `static_friction` < `dynamic_friction`

   No value will be configured to use default values. */
  CompliantMaterial(double youngs_modulus, double dissipation,
                    double static_friction, double dynamic_friction);

  /** Set the value for Young's modulus. Attempting to set a negative value will
   throw an exception. Returns a reference to `*this` so that multiple
   invocations of `set` can be chained together.
   @param  value  The Young's modulus value with units of pascals. */
  CompliantMaterial& set_youngs_modulus(double value);

  /** Extract the value for Young's modulus with an optional default value. The
   default value is checked in Debug build. */
  double youngs_modulus(double default_value = kDefaultYoungsModulus) const {
    DRAKE_ASSERT(default_value >= 0);
    return youngs_modulus_.value_or(default_value);
  }

  /** Reports if the elasticity value is configured to use a default. A newly
   constructed instance will report true. Furthermore, it must
   return true if set_youngs_modulus_to_default() has been called and false if
   set_youngs_modulus() has been called. */
  bool youngs_modulus_is_default() const {
    return youngs_modulus_ == std::nullopt;
  }

  /** Reset the elasticity value to report the default value. */
  void set_youngs_modulus_to_default() { youngs_modulus_ = std::nullopt; }

  /** Set the dissipation value. Attempting to set a negative value will throw
   an exception. Returns a reference to `*this` so that multiple invocations
   of `set` can be chained together.  */
  CompliantMaterial& set_dissipation(double value);

  /** Extract the dissipation value with an optional default value. The default
   value is checked in Debug build.
   @param  value  The dissipation value with units of 1/velocity, i.e., s/m. */
  double dissipation(double default_value = kDefaultDissipation) const {
    DRAKE_ASSERT(default_value >= 0);
    return dissipation_.value_or(default_value);
  }

  /** Reports if the dissipation value is configured to use a default. A newly
   constructed instance will report true. Furthermore, it must
   return true if set_dissipation_to_default() has been called and false if
   set_dissipation() has been called. */
  bool dissipation_is_default() const { return dissipation_ == std::nullopt; }

  /** Reset the dissipation value to report the default value. */
  void set_dissipation_to_default() { dissipation_ = std::nullopt; }

  /** Sets *both* coefficients of friction to the same value.
   @throws std::exception if the value is negative.  Returns a reference to
   `*this` so that multiple invocations of `set` can be chained together. */
  CompliantMaterial& set_friction(double value);

  /** Sets the two coefficients of friction. The `dynamic_friction` values must
   be less than or equal to the `static_friction`. An exception will be thrown
   if `dynamic_friction` > `static_friction` or if either value is negative.
   Returns a reference to `*this` so that multiple invocations of `set` can be
   chained together. */
  CompliantMaterial& set_friction(double static_friction,
                                  double dynamic_friction);

  /** Extract the static friction coefficient with an optional default value.
   The default value is checked for non-negativity in Debug build.

   If the caller provides a default value for accessing static and dynamic
   friction, it is the callers responsibility to confirm that the dynamic
   friction default is less than or equal to the static friction default.
   */
  double static_friction(double default_value = kDefaultStaticFriction) const {
    DRAKE_ASSERT(default_value >= 0);
    return static_friction_.value_or(default_value);
  }

  /** Extract the dynamic friction coefficient with an optional default value.
   The default value is checked for non-negativity in Debug build.

   If the caller provides a default value for accessing static and dynamic
   friction, it is the callers responsibility to confirm that the dynamic
   friction default is less than or equal to the static friction default.
   */
  double dynamic_friction(
      double default_value = kDefaultDynamicFriction) const {
    DRAKE_ASSERT(default_value >= 0);
    return dynamic_friction_.value_or(default_value);
  }

  /** Reports if the friction values are configured to use a default. A newly
   constructed instance will report true. Furthermore, it must
   return true if set_friction_to_default() has been called and false if
   set_friction() has been called. */
  bool friction_is_default() const {
    // NOTE: Friction values can only be set in tandem; so if one is default,
    // both must be default.
    return static_friction_ == std::nullopt;
  }

  /** Reset both friction coefficient values to report the default value. */
  void set_friction_to_default() {
    static_friction_ = std::nullopt;
    dynamic_friction_ = std::nullopt;
  }

  static const double kDefaultYoungsModulus;
  static const double kDefaultDissipation;
  static const double kDefaultStaticFriction;
  static const double kDefaultDynamicFriction;

 private:
  // Confirms two properties on the friction coefficient pair:
  //  1. Both values non-negative.
  //  2. static_friction >= dynamic_friction.
  // Throws std::runtime_error on failure of these tests.
  static void ThrowForBadFriction(double static_friction,
                                  double dynamic_friction);

  std::optional<double> youngs_modulus_;
  std::optional<double> dissipation_;
  std::optional<double> static_friction_;
  std::optional<double> dynamic_friction_;
};

}  // namespace systems
}  // namespace drake
