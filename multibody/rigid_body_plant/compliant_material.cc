#include "drake/multibody/rigid_body_plant/compliant_material.h"

#include <string>

namespace drake {
namespace systems {

using std::to_string;

// These values must *always* be non-negative and static friction must be
// greater than or equal to dynamic friction. If *any* of these values get
// changed, the class doxygen should echo the new values as well as modifying
// the corresponding unit test.
const double CompliantMaterial::kDefaultYoungsModulus = 1e8;  // Pa
const double CompliantMaterial::kDefaultDissipation = 0.32;  // s/m
const double CompliantMaterial::kDefaultStaticFriction = 0.9;
const double CompliantMaterial::kDefaultDynamicFriction = 0.5;

CompliantMaterial::CompliantMaterial(double youngs_modulus, double dissipation,
                                     double static_friction,
                                     double dynamic_friction) {
  // Simple expedient to take advantage of all the built-in validation.
  set_youngs_modulus(youngs_modulus);
  set_dissipation(dissipation);
  set_friction(static_friction, dynamic_friction);
}

CompliantMaterial& CompliantMaterial::set_youngs_modulus(double value) {
  if (value <= 0) {
    throw std::runtime_error(
        "Young's modulus value must be non-negative. Given " +
        to_string(value));
  }
  youngs_modulus_ = value;
  return *this;
}

CompliantMaterial& CompliantMaterial::set_dissipation(double value) {
  if (value < 0) {
    throw std::runtime_error(
        "Dissipation value must be non-negative. Given " + to_string(value));
  }
  dissipation_ = value;
  return *this;
}

CompliantMaterial& CompliantMaterial::set_friction(double value) {
  ThrowForBadFriction(value, value);
  static_friction_ = dynamic_friction_ = value;
  return *this;
}

CompliantMaterial& CompliantMaterial::set_friction(double static_friction,
                                              double dynamic_friction) {
  ThrowForBadFriction(static_friction, dynamic_friction);
  static_friction_ = static_friction;
  dynamic_friction_ = dynamic_friction;
  return *this;
}

void CompliantMaterial::ThrowForBadFriction(double static_friction,
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

}  // namespace systems
}  // namespace drake
