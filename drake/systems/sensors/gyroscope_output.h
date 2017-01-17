#pragma once

#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace sensors {

/// Specializes BasicVector with accessors and setters that are useful for
/// consumers of Gyroscope's output.
template <typename T>
class GyroscopeOutput : public BasicVector<T> {
 public:
  /// Default constructor.  Sets all rows to zero.
  GyroscopeOutput();

  /// @name Getters and Setters
  //@{
  /// Returns the rotational acceleration about the +X axis of the sensor's
  /// frame.
  const T& get_rotational_accel_x() const;

  /// Returns the rotational acceleration about the +Y axis of the sensor's
  /// frame.
  const T& get_rotational_accel_y() const;

  /// Returns the rotational acceleration about the +Z axis of the sensor's
  /// frame.
  const T& get_rotational_accel_z() const;
  //@}
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
