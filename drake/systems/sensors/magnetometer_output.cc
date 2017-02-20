#include "drake/systems/sensors/magnetometer_output.h"

namespace drake {
namespace systems {
namespace sensors {

const int MagnetometerOutputConstants::kIndexWx;
const int MagnetometerOutputConstants::kIndexWy;
const int MagnetometerOutputConstants::kIndexWz;

template <typename T>
MagnetometerOutput<T>::MagnetometerOutput() : BasicVector<double>(3) {
  this->SetFromVector(VectorX<double>::Zero(3));
}

template <typename T>
Vector3<T> MagnetometerOutput<T>::get_measurement() const {
  Vector3<T> rotational_velocity(
      BasicVector<T>::GetAtIndex(MagnetometerOutputConstants::kIndexWx),
      BasicVector<T>::GetAtIndex(MagnetometerOutputConstants::kIndexWy),
      BasicVector<T>::GetAtIndex(MagnetometerOutputConstants::kIndexWz));
  return rotational_velocity;
}

template <typename T>
MagnetometerOutput<T>* MagnetometerOutput<T>::DoClone() const {
  auto result = new MagnetometerOutput;
  result->set_value(this->get_value());
  return result;
}

template class MagnetometerOutput<double>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
