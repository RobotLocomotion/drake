#include "drake/multibody/plant/coulomb_friction.h"

#include <fmt/format.h>
#include <fmt/ostream.h>

namespace drake {
namespace multibody {

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
  using std::logic_error;
  if (dynamic_friction < 0) {
    throw logic_error(fmt::format(
        "The given dynamic friction is negative: {}", dynamic_friction));
  }
  if (static_friction < 0) {
    throw logic_error(fmt::format(
        "The given static friction is negative: {}", static_friction));
  }
  if (dynamic_friction > static_friction) {
    throw logic_error(fmt::format(
        "The given dynamic friction ({}) is greater than the given static "
        "friction ({}); dynamic friction must be less than or equal to static "
        "friction.", dynamic_friction, static_friction));
  }
}

template <typename T>
boolean<T> CoulombFriction<T>::operator==(const CoulombFriction& other) const {
  return static_friction() == other.static_friction() &&
      dynamic_friction() == other.dynamic_friction();
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::CoulombFriction);
