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

  /// @name Getters and Setters
  //@{
  /// Returns the measured distance when the sensor is in the provided @p theta
  /// and @p phi configuration.
  ///
  /// @param[in] theta The theta angle of the sensor in the sensor's frame. This
  /// is the rotation about the +Z axis (right-hand rule), zero points down the
  /// +X axis.
  ///
  /// @param[in] phi The phi angle of the sensor in the sensor's frame. This is
  /// the rotation about the +Y axis (right-hand rule). A value of zero points
  /// down the +X axis. A value of M_PI / 2 points down the -Z axis.
  ///
  /// @return The measured distance when the sensor is at @p theta and @p phi.
  ///
  /// @throws std::runtime_error if @p theta or @p phi are invalid, which occurs
  /// when they fall out of the sensor's min / max rotation range or if no
  /// measurement was taken at the specified sensor angles.
  double GetDistance(double theta, double phi) const;

  //@}

 private:
  const DepthSensorSpecification& spec_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
