#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace sensors {

/// Defines the semantics of each index in GyroscopeOutput's vector.
struct GyroscopeOutputConstants {
  static const int kIndexWx{0};
  static const int kIndexWy{1};
  static const int kIndexWz{2};
};

/// Specializes BasicVector with accessors and setters that are useful for
/// consumers of Gyroscope's output.
template <typename T>
class GyroscopeOutput : public BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GyroscopeOutput);

  /// Default constructor.  Sets all rows to zero.
  GyroscopeOutput();

  /// @name Accessors
  /// @{

  /// Returns a Vector3<T> containing the rotational velocity in the sensor's
  /// frame. The ordering of the values in this 3-vector are
  /// `[roll, pitch, yaw]`.
  Vector3<T> get_rotational_velocities() const;
  /// @}

 protected:
  GyroscopeOutput* DoClone() const override;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
