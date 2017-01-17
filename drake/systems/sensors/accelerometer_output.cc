#include "drake/systems/sensors/accelerometer_output.h"

#include <cmath>

#include "drake/systems/sensors/accelerometer.h"

namespace drake {
namespace systems {
namespace sensors {

template <typename T>
AccelerometerOutput<T>::AccelerometerOutput()
    : BasicVector<double>(Accelerometer::kNumMeasurements) {
  this->SetFromVector(VectorX<double>::Zero(Accelerometer::kNumMeasurements));
}

template <typename T>
const T& AccelerometerOutput<T>::get_accel_x() const {
  return BasicVector<T>::GetAtIndex(kAccelXIndex);
}

template <typename T>
const T& AccelerometerOutput<T>::get_accel_y() const {
  return BasicVector<T>::GetAtIndex(kAccelYIndex);
}

template <typename T>
const T& AccelerometerOutput<T>::get_accel_z() const {
  return BasicVector<T>::GetAtIndex(kAccelZIndex);
}

template class AccelerometerOutput<double>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
