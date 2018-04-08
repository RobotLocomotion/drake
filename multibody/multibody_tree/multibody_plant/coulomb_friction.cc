#include "drake/multibody/multibody_tree/multibody_plant/coulomb_friction.h"

#include <string>

namespace drake {
namespace multibody {
namespace multibody_plant {

CoulombFriction::CoulombFriction(
                                     double static_friction,
                                     double dynamic_friction) {
  ThrowForBadFriction(static_friction, dynamic_friction);
  static_friction_ = static_friction;
  dynamic_friction_ = dynamic_friction;
}

void CoulombFriction::ThrowForBadFriction(double static_friction,
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

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
