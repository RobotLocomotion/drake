#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

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
   not or if the value is negative. */
  void set_friction(double static_friction, double dynamic_friction);

  double static_friction() const {
    return static_friction_.value_or(kDefaultStaticFriction);
  }
  double dynamic_friction() const {
    return dynamic_friction_.value_or(kDefaultDynamicFriction);
  }

  /** Sets the global default parameters based on the given value. The default
   value for a particular parameter will not change if the corresponding
   parameter field in the input instance has not been explicitly set. */
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

}  // namespace systems
}  // namespace drake
