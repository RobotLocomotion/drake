#include "drake/multibody/multibody_tree/multibody_plant/coulomb_friction.h"

#include <sstream>

#include "drake/common/default_scalars.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

template <typename T>
CoulombFriction<T>::CoulombFriction(const T& static_friction,
                                    const T& dynamic_friction) {
  ThrowForBadFriction(static_friction, dynamic_friction);
  static_friction_ = static_friction;
  dynamic_friction_ = dynamic_friction;
}

template <typename T>
void CoulombFriction<T>::ThrowForBadFriction(const T& static_friction,
                                             const T& dynamic_friction) {
  using std::runtime_error;
  using std::stringstream;
  if (dynamic_friction < 0) {
    stringstream stream;
    stream << "The given dynamic friction is negative: "  << dynamic_friction;
    throw runtime_error(stream.str());
  }
  if (static_friction < 0) {
    stringstream stream;
    stream << "The given static friction is negative: " << static_friction;
    throw runtime_error(stream.str());
  }
  if (dynamic_friction > static_friction) {
    stringstream stream;
    stream << "The given dynamic friction (" << dynamic_friction <<
           ") is greater than the given static friction (" << static_friction <<
           "); dynamic friction must be less than or equal to static friction.";
    throw runtime_error(stream.str());
  }
}

template <typename T>
Bool<T> CoulombFriction<T>::operator==(const CoulombFriction& other) const {
  return static_friction() == other.static_friction() &&
      dynamic_friction() == other.dynamic_friction();
}

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::multibody_plant::CoulombFriction);
