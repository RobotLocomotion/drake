#pragma once

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace sensors {

/// Defines the semantics of each index in MagnetometerOutput's vector.
struct MagnetometerOutputConstants {
  static const int kIndexWx{0};
  static const int kIndexWy{1};
  static const int kIndexWz{2};
};

/// Specializes BasicVector with accessors and setters that are useful for
/// consumers of Magnetometer's output.
template <typename T>
class MagnetometerOutput : public BasicVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MagnetometerOutput);

  /// Default constructor.  Sets all rows to zero.
  MagnetometerOutput();

  /// @name Accessors
  /// @{
  /// Returns a Vector3<T> containing the magnetometer's measurement.
  Vector3<T> get_measurement() const;
  /// @}

 protected:
  MagnetometerOutput* DoClone() const override;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
