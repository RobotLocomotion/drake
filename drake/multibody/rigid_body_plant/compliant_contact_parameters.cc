#include "drake/multibody/rigid_body_plant/compliant_contact_parameters.h"

#include <string>

namespace drake {
namespace systems {

using std::to_string;

double CompliantContactParameters::kDefaultStiffness = 20000.0;
double CompliantContactParameters::kDefaultDissipation = 2;
double CompliantContactParameters::kDefaultStaticFriction = 0.9;
double CompliantContactParameters::kDefaultDynamicFriction = 0.5;

void CompliantContactParameters::set_stiffness(double value) {
  if (value <= 0) {
    throw std::runtime_error(
        "Stiffness value must be non-negative. Given " + to_string(value));
  }
  stiffness_ = value;
}

void CompliantContactParameters::set_dissipation(double value) {
  if (value < 0) {
    throw std::runtime_error(
        "Dissipation value must be non-negative. Given " + to_string(value));
  }
  dissipation_ = value;
}

void CompliantContactParameters::set_friction(double value) {
  ThrowForBadFriction(value, value);
  static_friction_ = dynamic_friction_ = value;
}

void CompliantContactParameters::set_friction(double static_friction,
                                              double dynamic_friction) {
  ThrowForBadFriction(static_friction, dynamic_friction);
  static_friction_ = static_friction;
  dynamic_friction_ = dynamic_friction;
}

void CompliantContactParameters::ThrowForBadFriction(double static_friction,
                                                     double dynamic_friction) {
  using std::to_string;
  using std::runtime_error;
  if (dynamic_friction < 0) {
    throw runtime_error("Given dynamic friction is negative: " +
                        to_string(dynamic_friction));
  }
  if (static_friction < 0) {
    throw runtime_error("Given static friction is negative: " +
                        to_string(dynamic_friction));
  }
  if (dynamic_friction > static_friction) {
    throw std::runtime_error("Given dynamic friction (" +
                             to_string(dynamic_friction) +
                             ") is greater than given static friction (" +
                             to_string(static_friction) +
                             "). Must be less or equal.");
  }
}

void CompliantContactParameters::SetDefaultValues(
    const CompliantContactParameters &values) {
  // NOTE: This doesn't validate the friction values; it relies on the friction
  // setters on the instance to have already done this.
  kDefaultStiffness = values.stiffness();
  kDefaultDissipation = values.dissipation();
  kDefaultStaticFriction = values.static_friction();
  kDefaultDynamicFriction = values.dynamic_friction();
}

}  // namespace systems
}  // namespace drake
