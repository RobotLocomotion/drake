#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace sensors {

struct AccelerometerOutputConstants {
  static const int kAccelXIndex{0};
  static const int kAccelYIndex{1};
  static const int kAccelZIndex{2};
};

/// Specializes BasicVector with accessors and setters that are useful for
/// consumers of Accelerometer's output.
template <typename T>
class AccelerometerOutput : public BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AccelerometerOutput)

  /// Default constructor.  Sets all rows to zero.
  AccelerometerOutput();

  /// @name Accessors
  //@{

  /// Returns a Vector3<T> containing the linear acceleration in the sensor's
  /// frame. The ordering of the values in this 3-vector are `[x, y, z]`.
  Vector3<T> get_accel() const;
  //@}

 protected:
  AccelerometerOutput* DoClone() const override;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
