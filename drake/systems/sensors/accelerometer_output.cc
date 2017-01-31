#include "drake/systems/sensors/accelerometer_output.h"

#include <cmath>

#include "drake/systems/sensors/accelerometer.h"

namespace drake {
namespace systems {
namespace sensors {

const int AccelerometerOutputConstants::kAccelXIndex;
const int AccelerometerOutputConstants::kAccelYIndex;
const int AccelerometerOutputConstants::kAccelZIndex;

template <typename T>
AccelerometerOutput<T>::AccelerometerOutput() : BasicVector<double>(3) {
  this->SetFromVector(VectorX<double>::Zero(3));
}

template <typename T>
Vector3<T> AccelerometerOutput<T>::get_accel() const {
  // TODO(liang.fok) Optimize the following code by re-interpreting the three
  // acceleration values, which are already sitting sequentially in
  // BasicVector's memory, as a Vector3<T> with no copying.
  Vector3<T> acceleration(
      BasicVector<T>::GetAtIndex(AccelerometerOutputConstants::kAccelXIndex),
      BasicVector<T>::GetAtIndex(AccelerometerOutputConstants::kAccelYIndex),
      BasicVector<T>::GetAtIndex(AccelerometerOutputConstants::kAccelZIndex));
  return acceleration;
}

template class AccelerometerOutput<double>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
