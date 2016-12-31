#include "drake/systems/sensors/depth_sensor_output.h"

#include <cmath>
#include <vector>

#include "drake/systems/sensors/depth_sensor.h"

using std::runtime_error;
using std::to_string;

namespace drake {
namespace systems {
namespace sensors {

template <typename T>
DepthSensorOutput<T>::DepthSensorOutput(const DepthSensorSpecification& spec)
    : BasicVector<double>(spec.num_depth_readings()), spec_(spec) {
  this->SetFromVector(VectorX<double>::Zero(spec_.num_depth_readings()));
  point_cloud_.frame_id = spec_.frame_id();
  point_cloud_.seq = point_cloud_seq_;

  // Allocate the maximum amount of memory for the point cloud. This occurs when
  // every raycast results in a valid measurement.
  point_cloud_.n_points = spec_.num_depth_readings();

  point_cloud_.points.clear();
  for (int i = 0; i < spec_.num_depth_readings(); ++i) {
    std::vector<float> xyz_point;
    xyz_point.resize(3);
    point_cloud_.points.push_back(xyz_point);
  }

  // There is only one channel. It contains the distance measurements.
  point_cloud_.n_channels = 1;
  point_cloud_.channel_names.clear();
  point_cloud_.channel_names.push_back("distance");

  point_cloud_.channels.clear();
  std::vector<float> distance_channel;
  distance_channel.resize(spec_.num_depth_readings());
  point_cloud_.channels.push_back(distance_channel);
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
const bot_core::pointcloud_t& DepthSensorOutput<T>::GetPointCloud() const {
  return point_cloud_;
}

template <typename T>
void DepthSensorOutput<T>::SetDistances(
    double time, const Eigen::Ref<const VectorX<T>>& value) {
  BasicVector<T>::SetFromVector(value);
  point_cloud_.utime = time * 1e6;
  point_cloud_.seq = point_cloud_seq_++;

  int point_cloud_index = 0;
  for (int yaw_index = 0; yaw_index < spec_.num_yaw_values(); ++yaw_index) {
    for (int pitch_index = 0; pitch_index < spec_.num_pitch_values();
         ++pitch_index) {
      double distance = GetDistance(yaw_index, pitch_index);

      if (distance != DepthSensor::kError && distance != DepthSensor::kTooFar &&
          distance != DepthSensor::kTooClose) {
        double yaw = spec_.min_yaw() + yaw_index * spec_.yaw_increment();
        double pitch =
            spec_.min_pitch() + pitch_index * spec_.pitch_increment();

        // Compute the Cartesian location of the collision. This is done using
        // the same equations that convert from spherical coordinates to
        // Cartesian coordinates.
        double x = cos(pitch) * cos(yaw);
        double y = cos(pitch) * sin(yaw);
        double z = sin(pitch);

        point_cloud_.points.at(point_cloud_index).at(0) = x;
        point_cloud_.points.at(point_cloud_index).at(1) = y;
        point_cloud_.points.at(point_cloud_index).at(2) = z;

        point_cloud_.channels.at(0).at(point_cloud_index) =
            static_cast<float>(distance);

        point_cloud_index++;
      }
    }
  }

  point_cloud_.n_points = point_cloud_index;
}

template class DepthSensorOutput<double>;

}  // namespace sensors
}  // namespace systems
}  // namespace drake
