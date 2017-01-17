#include "drake/systems/sensors/gyroscope_output.h"

#include <cmath>

#include "drake/systems/sensors/gyroscope.h"

namespace drake {
namespace systems {
namespace sensors {

template <typename T>
GyroscopeOutput<T>::GyroscopeOutput()
    : BasicVector<double>(Gyroscope::kNumMeasurements) {
  this->SetFromVector(VectorX<double>::Zero(Gyroscope::kNumMeasurements));
}

template <typename T>
const T& GyroscopeOutput<T>::get_rotational_accel_x() const {
  return BasicVector<T>::GetAtIndex(0);
}

template <typename T>
const T& GyroscopeOutput<T>::get_rotational_accel_y() const {
  return BasicVector<T>::GetAtIndex(1);
}

template <typename T>
const T& GyroscopeOutput<T>::get_rotational_accel_z() const {
  return BasicVector<T>::GetAtIndex(2);
}

template class GyroscopeOutput<double>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
