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
const Vector3<T>& AccelerometerOutput<T>::get_accel() const {
  acceleration_ <<
      BasicVector<T>::GetAtIndex(AccelerometerOutputConstants::kAccelXIndex),
      BasicVector<T>::GetAtIndex(AccelerometerOutputConstants::kAccelYIndex),
      BasicVector<T>::GetAtIndex(AccelerometerOutputConstants::kAccelZIndex);
  return acceleration_;
}

template class AccelerometerOutput<double>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
