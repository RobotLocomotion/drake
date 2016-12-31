#include "drake/systems/sensors/depth_sensor_output.h"

#include <cmath>

using std::runtime_error;
using std::to_string;

namespace drake {
namespace systems {
namespace sensors {

template <typename T>
DepthSensorOutput<T>::DepthSensorOutput(const DepthSensorSpecification& spec)
    : BasicVector<double>(spec.num_depth_readings()), spec_(spec) {
  this->SetFromVector(VectorX<double>::Zero(spec_.num_depth_readings()));
}

template <typename T>
double DepthSensorOutput<T>::GetDistance(int yaw_index, int pitch_index) const {
  if (yaw_index < 0) {
    throw runtime_error("DepthSensorOutput: GetDistance: yaw_index (" +
                        to_string(yaw_index) +
                        ") must be greater than or equal to zero.");
  }

  if (yaw_index >= spec_.num_yaw_values()) {
    throw runtime_error("DepthSensorOutput: GetDistance: yaw_index (" +
                        to_string(yaw_index) +
                        ") must be less than the number of columns in the "
                        "depth image (" +
                        to_string(spec_.num_yaw_values()) + ").");
  }

  if (pitch_index < 0) {
    throw runtime_error("DepthSensorOutput: GetDistance: pitch_index (" +
                        to_string(pitch_index) +
                        ") must be greater than or equal to zero.");
  }

  if (pitch_index >= spec_.num_pitch_values()) {
    throw runtime_error(
        "DepthSensorOutput: GetDistance: pitch_index (" +
        to_string(pitch_index) +
        ") must be less than the number of rows in the depth image (" +
        to_string(spec_.num_pitch_values()) + ").");
  }

  return this->GetAtIndex(pitch_index * spec_.num_yaw_values() + yaw_index);
}

template class DepthSensorOutput<double>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
