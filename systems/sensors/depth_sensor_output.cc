#include "drake/systems/sensors/depth_sensor_output.h"

#include <cmath>
#include <limits>
#include <vector>

using std::runtime_error;
using std::to_string;

using Eigen::Matrix3Xd;

namespace drake {
namespace systems {
namespace sensors {

template <typename T>
DepthSensorOutput<T>::DepthSensorOutput(const DepthSensorSpecification& spec)
    : BasicVector<double>(VectorX<double>::Zero(spec.num_depth_readings())),
      spec_(spec) {}

template <typename T>
double DepthSensorOutput<T>::GetTooFarDistance() {
  return std::numeric_limits<double>::infinity();
}

template <typename T>
bool DepthSensorOutput<T>::IsTooFar(double distance) {
  return distance == std::numeric_limits<double>::infinity();
}

template <typename T>
double DepthSensorOutput<T>::GetTooCloseDistance() {
  return 0;
}

template <typename T>
bool DepthSensorOutput<T>::IsTooClose(double distance) {
  return distance == 0;
}

template <typename T>
double DepthSensorOutput<T>::GetErrorDistance() {
  return std::numeric_limits<double>::quiet_NaN();
}

template <typename T>
bool DepthSensorOutput<T>::IsError(double distance) {
  return std::isnan(distance);
}

template <typename T>
bool DepthSensorOutput<T>::IsValid(double distance) {
  return !IsTooFar(distance) && !IsTooClose(distance) && !IsError(distance);
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

template <typename T>
int DepthSensorOutput<T>::GetNumValidDistanceMeasurements() const {
  int result = 0;
  for (int i = 0; i < spec_.num_depth_readings(); ++i) {
    const double distance = BasicVector<T>::GetAtIndex(i);
    if (IsValid(distance)) {
      ++result;
    }
  }
  return result;
}

template <typename T>
Matrix3Xd DepthSensorOutput<T>::GetPointCloud() const {
  const int num_depth_measurements = GetNumValidDistanceMeasurements();
  Matrix3Xd result = Matrix3Xd::Zero(3, num_depth_measurements);

  int point_cloud_index = 0;
  for (int yaw_index = 0; yaw_index < spec_.num_yaw_values(); ++yaw_index) {
    for (int pitch_index = 0; pitch_index < spec_.num_pitch_values();
         ++pitch_index) {
      const double distance = GetDistance(yaw_index, pitch_index);
      if (IsValid(distance)) {
        const double yaw = spec_.min_yaw() + yaw_index * spec_.yaw_increment();
        const double pitch =
            spec_.min_pitch() + pitch_index * spec_.pitch_increment();

        // Compute the Cartesian location of the collision. This is done using
        // the same equations that convert from spherical coordinates to
        // Cartesian coordinates.
        const double x = distance * cos(pitch) * cos(yaw);
        const double y = distance * cos(pitch) * sin(yaw);
        const double z = distance * sin(pitch);

        DRAKE_ASSERT(point_cloud_index < num_depth_measurements);
        result(0, point_cloud_index) = x;
        result(1, point_cloud_index) = y;
        result(2, point_cloud_index) = z;

        point_cloud_index++;
      }
    }
  }

  DRAKE_ASSERT(point_cloud_index == num_depth_measurements);

  return result;
}

template <typename T>
DepthSensorOutput<T>* DepthSensorOutput<T>::DoClone() const {
  return new DepthSensorOutput(spec_);
}

template class DepthSensorOutput<double>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
