#include "drake/systems/sensors/gyroscope_output.h"

#include <cmath>

#include "drake/systems/sensors/gyroscope.h"

namespace drake {
namespace systems {
namespace sensors {

const int GyroscopeOutputConstants::kRollIndex;
const int GyroscopeOutputConstants::kPitchIndex;
const int GyroscopeOutputConstants::kYawIndex;

template <typename T>
GyroscopeOutput<T>::GyroscopeOutput() : BasicVector<double>(3) {
  this->SetFromVector(VectorX<double>::Zero(3));
}

template <typename T>
Vector3<T> GyroscopeOutput<T>::get_rotational_velocities() const {
  // TODO(liang.fok) Optimize the following code by re-interpreting the three
  // rotational velocity values, which are already sitting sequentially in
  // BasicVector's memory, as a Vector3<T> with no copying.
  Vector3<T> rotational_velocity(
      BasicVector<T>::GetAtIndex(GyroscopeOutputConstants::kRollIndex),
      BasicVector<T>::GetAtIndex(GyroscopeOutputConstants::kPitchIndex),
      BasicVector<T>::GetAtIndex(GyroscopeOutputConstants::kYawIndex));
  return rotational_velocity;
}

template class GyroscopeOutput<double>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
