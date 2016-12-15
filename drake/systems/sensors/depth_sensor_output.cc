#include "drake/systems/sensors/depth_sensor_output.h"

#include <cmath>

namespace drake {
namespace systems {
namespace sensors {

template <typename T>
DepthSensorOutput<T>::DepthSensorOutput(const DepthSensorSpecification& spec)
    : BasicVector<double>(spec.num_depth_readings()), spec_(spec) {
  this->SetFromVector(VectorX<double>::Zero(spec_.num_depth_readings()));
}

template <typename T>
double DepthSensorOutput<T>::GetDistance(double theta, double phi) const {
  if (phi < spec_.min_phi())
    throw std::runtime_error("phi is less than minimum phi.");

  if (phi > spec_.max_phi())
    throw std::runtime_error("phi is greater than maximum phi.");

  if (theta < spec_.min_theta())
    throw std::runtime_error("theta is less than minimum theta.");

  if (theta > spec_.max_theta())
    throw std::runtime_error("theta is greater than maximum theta.");

  double r = remainder((phi - spec_.min_phi()), spec_.phi_increment());
  if (r > 1e-10) {
    throw std::runtime_error("No measurement at phi = " + std::to_string(phi) +
                             " was taken, remainder = " + std::to_string(r));
  }
  int phi_index =
      static_cast<int>((phi - spec_.min_phi()) / spec_.phi_increment());

  r = remainder(theta - spec_.min_theta(), spec_.theta_increment());
  if (r > 1e-10) {
    throw std::runtime_error("No measurement at theta = " +
                             std::to_string(theta) +
                             " was taken, remainder = " + std::to_string(r));
  }
  int theta_index =
      static_cast<int>((theta - spec_.min_theta()) / spec_.phi_increment());

  return this->GetAtIndex(phi_index * spec_.num_phi_values() + theta_index);
}

template class DepthSensorOutput<double>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
