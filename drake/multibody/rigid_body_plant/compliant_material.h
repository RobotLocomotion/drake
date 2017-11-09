#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace systems {

/** The set of per-object compliant material properties. It attempts to model
 the material properties of a single object. The material properties include:
   - stiffness
   - dissipation
   - coefficients of friction (static and dynamic).

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

 Using the stiffness property to provide a concrete example:

 ```
 // Constructor sets all properties to be default configured.
 CompliantMaterial material;

 material.stiffness();        // Reports the hard-coded default value.
 material.stiffness(1.2);     // Reports the provided default value, 1.2.

 material.set_stiffness(10);  // stiffness is no longer default.

 material.stiffness();        // Reports the explicit value, 10.
 material.stiffness(1.2);     // Reports the explicit value, 10.
 ```

 See @ref drake_contacts for semantics of these properties for dynamics. */
class CompliantMaterial {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CompliantMaterial)
  CompliantMaterial() = default;

  void set_stiffness(double value);
  double stiffness(double default_value = kDefaultStiffness) const {
    return stiffness_.value_or(default_value);
  }
  bool stiffness_is_default() const { return stiffness_ == nullopt; }

  void set_dissipation(double value);
  double dissipation(double default_value = kDefaultDissipation) const {
    return dissipation_.value_or(default_value);
  }
  bool dissipation_is_default() const { return dissipation_ == nullopt; }

  /** Sets *both* coefficients of friction to the same value. Throws an
   exception if the value is negative. */
  void set_friction(double value);

  /** Sets the two coefficients of friction. The `dynamic_friction` values must
   be less than or equal to the `static_friction`. An error will be thrown if
   `dynamic_friction` > `static_friction` or if either value is negative. */
  void set_friction(double static_friction, double dynamic_friction);

  double static_friction(double default_value = kDefaultStaticFriction) const {
    return static_friction_.value_or(default_value);
  }
  double dynamic_friction(
      double default_value = kDefaultDynamicFriction) const {
    return dynamic_friction_.value_or(default_value);
  }

  bool friction_is_default() const {
    // NOTE: Friction values can only be set in tandem; so if one is default,
    // both must be default.
    return static_friction_ == nullopt;
  }

 private:
  // Confirms two properties on the friction coefficient pair:
  //  1. Both values non-negative.
  //  2. static_friction >= dynamic_friction.
  // Throws std::runtime_error on failure of these tests.
  static void ThrowForBadFriction(double static_friction,
                                  double dynamic_friction);

  optional<double> stiffness_;
  optional<double> dissipation_;
  optional<double> static_friction_;
  optional<double> dynamic_friction_;

  static const double kDefaultStiffness;
  static const double kDefaultDissipation;
  static const double kDefaultStaticFriction;
  static const double kDefaultDynamicFriction;
};

}  // namespace systems
}  // namespace drake
