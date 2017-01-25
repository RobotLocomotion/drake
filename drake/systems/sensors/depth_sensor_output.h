#pragma once

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

  // Non-copyable.
  /// @name Deleted Copy/Move Operations
  /// DepthSensorOutput is neither copyable nor moveable.
  ///@{
  explicit DepthSensorOutput(const DepthSensorOutput&) = delete;
  DepthSensorOutput& operator=(const DepthSensorOutput&) = delete;
  ///@}

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

  /// Returns the number of valid distance measurements within this output. This
  /// excludes the following depth values:
  ///
  ///   - DepthSensor::kError
  ///   - DepthSensor::kTooFar
  ///   - DepthSensor::kTooClose
  ///
  int GetNumValidDistanceMeasurements() const;

  /// Returns a point cloud based on the depth image contained within this
  /// output. Both the depth image and resulting point cloud are defined in the
  /// sensor's base frame.
  Eigen::Matrix3Xd GetPointCloud() const;

  //@}

 private:
  const DepthSensorSpecification& spec_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
