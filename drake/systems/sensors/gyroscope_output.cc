#include "drake/systems/sensors/gyroscope_output.h"

#include <cmath>

#include "drake/systems/sensors/gyroscope.h"

namespace drake {
namespace systems {
namespace sensors {

const int GyroscopeOutputConstants::kIndexWx;
const int GyroscopeOutputConstants::kIndexWy;
const int GyroscopeOutputConstants::kIndexWz;

template <typename T>
GyroscopeOutput<T>::GyroscopeOutput() : BasicVector<double>(3) {
  this->SetFromVector(VectorX<double>::Zero(3));
}

template <typename T>
Vector3<T> GyroscopeOutput<T>::get_rotational_velocities() const {
  Vector3<T> rotational_velocity(
      BasicVector<T>::GetAtIndex(GyroscopeOutputConstants::kIndexWx),
      BasicVector<T>::GetAtIndex(GyroscopeOutputConstants::kIndexWy),
      BasicVector<T>::GetAtIndex(GyroscopeOutputConstants::kIndexWz));
  return rotational_velocity;
}

template class GyroscopeOutput<double>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
