#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace tinyxml2 {
class XMLElement;
}

namespace drake {
namespace systems {

/** The set of per-object compliant contact parameters. It attempts to model
 the material properties of a single object. The material properties include:
   - stiffness
   - dissipation
   - coefficients of friction (static and dynamic).
 See @ref drake_contacts for details on these parameters. */
class CompliantContactParameters {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CompliantContactParameters)
  CompliantContactParameters() = default;

  void set_stiffness(double value);
  double stiffness() const { return stiffness_.value_or(kDefaultStiffness); }
  void set_dissipation(double value);
  double dissipation() const {
    return dissipation_.value_or(kDefaultDissipation);
  }

  /** Sets *both* coefficients of friction to the same value. Throws an
   exception if the value is negative. */
  void set_friction(double value);

  /** Sets the two coefficients of friction. The `dynamic_friction` values must
   be less than or equal to the `static_friction`. An error will be thrown if
   not or if the value is negative.. */
  void set_friction(double static_friction, double dynamic_friction);

  double static_friction() const {
    return static_friction_.value_or(kDefaultStaticFriction);
  }
  double dynamic_friction() const {
    return dynamic_friction_.value_or(kDefaultDynamicFriction);
  }

  /** Sets the default parameters based on the given value. Any values that have
   *not* been explicitly set in the input instance will remain unchanged. */
  static void SetDefaultValues(const CompliantContactParameters &values);

 private:
  // Confirms two properties on the coefficient pair:
  //  1. Both values non-negative.
  //  2. static_friction >= dynamic_friction.
  // Throws std::runtime_error on failure of these tests.
  static void ThrowForBadFriction(double static_friction,
                                  double dynamic_friction);

  optional<double> stiffness_;
  optional<double> dissipation_;
  optional<double> static_friction_;
  optional<double> dynamic_friction_;

  static double kDefaultStiffness;
  static double kDefaultDissipation;
  static double kDefaultStaticFriction;
  static double kDefaultDynamicFriction;
};

/// Instantiates a CompliantContactParameters instance from an XMLNode.
/// It ignores unrecognized elements, but throws an exception if a recognized
/// element's contents cannot be converted to a double. Omitted property
/// elements remain tied to the default parameter value.
/// If either friction coefficient is defined, _both_ must be defined.
/// Furthermore, the coefficient for static friction must be greater than or
/// equal to the dynamic friction and both must be non-negative.
///
/// Looks for the following tags in URDF and SDF:
///
/// ```xml
/// ...
/// <collision ...>
///   <geometry...>
///   </geometry>
///
///   <drake_compliance>
///     <stiffness>##</stiffness>
///     <dissipation>##</dissipation>
///     <static_friction>##</static_friction>
///     <dynamic_friction>##</dynamic_friction>
///   </drake_compliance>
///
/// </collision>
/// ...
/// ```
/// @param  node  The *parent* node which ostensibly contains a declaration of
///               drake compliance.
CompliantContactParameters ParseCollisionCompliance(tinyxml2::XMLElement* node);

}  // namespace systems
}  // namespace drake
