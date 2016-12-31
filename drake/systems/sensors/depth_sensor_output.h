#pragma once

#include "bot_core/pointcloud_t.hpp"

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/sensors/depth_sensor_specification.h"

namespace drake {
namespace systems {
namespace sensors {

/// Specializes BasicVector with specific getters and setters that are useful
/// for consumers of DepthSensor's output.
///
/// @see DepthSensor.
template <typename T>
class DepthSensorOutput : public BasicVector<T> {
 public:
  /// Default constructor.  Sets all rows to zero.
  ///
  /// @param[in] spec The sensor specification. A member variable alias is
  /// maintained. Thus, the lifespan of the reference object must exceed the
  /// lifespan of this class' instance.
  explicit DepthSensorOutput(const DepthSensorSpecification& spec);

  /// @name Getters and Setters
  //@{

  /// Returns the measured distance within the depth image at @p yaw_index and
  /// @p pitch_index.
  ///
  /// @param[in] yaw_index The index of the depth image pixel column containing
  /// the depth is to be returned. This value must be between zero and
  /// DepthSensor::get_num_pixel_cols().
  ///
  /// @param[in] pitch_index The index of the depth image pixel row containing
  /// the depth to be returned. This value must be between zero and
  /// DepthSensor::get_num_pixel_rows().
  ///
  /// @return The measured distance at @p yaw_index and @p pitch_index.
  ///
  /// @throws std::runtime_error if @p yaw_index or @p pitch_index is invalid.
  double GetDistance(int yaw_index, int pitch_index) const;

  /// Returns a point cloud in the sensor's base frame based on the depth image
  /// contained within this output.
  const bot_core::pointcloud_t& GetPointCloud() const;

  //@}

  /// This method first calls the parent class' BasicVector::SetFromVector()
  /// and then computes a point cloud based on the distance measurements and the
  /// depth sensor's specifications. The resulting point cloud is in the
  /// sensor's frame and can be obtained by calling
  /// GetPointCloudInSensorFrame().
  void SetDistances(double time, const Eigen::Ref<const VectorX<T>>& value);

 private:
  const DepthSensorSpecification& spec_;

  // The point cloud in the sensor's frame.
  bot_core::pointcloud_t point_cloud_;
  int point_cloud_seq_{0};
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
