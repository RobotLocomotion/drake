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
AccelerometerOutput<T>::AccelerometerOutput()
    : BasicVector<double>(Accelerometer::kNumDimensions) {
  this->SetFromVector(VectorX<double>::Zero(Accelerometer::kNumDimensions));
}

template <typename T>
const T& AccelerometerOutput<T>::get_accel_x() const {
  return BasicVector<T>::GetAtIndex(AccelerometerOutputConstants::kAccelXIndex);
}

template <typename T>
const T& AccelerometerOutput<T>::get_accel_y() const {
  return BasicVector<T>::GetAtIndex(AccelerometerOutputConstants::kAccelYIndex);
}

template <typename T>
const T& AccelerometerOutput<T>::get_accel_z() const {
  return BasicVector<T>::GetAtIndex(AccelerometerOutputConstants::kAccelZIndex);
}

template <typename T>
Vector3<T> AccelerometerOutput<T>::get_accel() const {
  Vector3<T> result(get_accel_x(), get_accel_y(), get_accel_z());
  return result;
}

template class AccelerometerOutput<double>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
