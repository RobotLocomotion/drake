#pragma once

#include <limits>

#include "drake/common/drake_copyable.h"
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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DepthSensorOutput)

  /// The depth value when an error occurs in obtaining the measurement.
  static constexpr double kError{std::numeric_limits<double>::quiet_NaN()};

  /// The depth value when the max sensing range is exceeded.
  static constexpr double kTooFar{std::numeric_limits<double>::infinity()};

  /// The depth value when the min sensing range is violated because the object
  /// being sensed is too close. Note that this
  /// <a href="http://www.ros.org/reps/rep-0117.html">differs from ROS</a>,
  /// which uses negative infinity in this scenario. Drake uses zero because it
  /// results in less devastating bugs when users fail to check for the lower
  /// limit being hit and using negative infinity does not prevent users from
  /// writing bad code.
  static constexpr double kTooClose{0};

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

 protected:
  DepthSensorOutput* DoClone() const override;

 private:
  const DepthSensorSpecification& spec_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
