#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/state_vector.h"

namespace drake {
namespace systems {

/// VectorBase is a pure abstract interface that real-valued signals
/// between Systems must satisfy. Classes that inherit from VectorBase
/// will typically provide names for the elements of the vector, and may also
/// provide other computations for the convenience of Systems handling the
/// signal. The vector is always a column vector.
///
/// @tparam T Must be a Scalar compatible with Eigen.
template <typename T>
class VectorBase : public StateVector<T> {
 public:
  virtual ~VectorBase() {}

  /// Sets the vector to the given value. After a.set_value(b.get_value()), a
  /// must be identical to b.
  /// May throw std::out_of_range if the new value has different dimensions
  /// than expected by the concrete class implementing VectorBase.
  DRAKE_DEPRECATED("Use SetFromVector instead.")
  virtual void set_value(const Eigen::Ref<const VectorX<T>>& value) = 0;

  /// Returns a column vector containing the entire value of the signal.
  DRAKE_DEPRECATED("Use a narrower accessor, or BasicVector if you can.")
  virtual Eigen::VectorBlock<const VectorX<T>> get_value() const = 0;

  /// Returns a reference that allows mutation of the values in this vector, but
  /// does not allow resizing the vector itself.
  DRAKE_DEPRECATED("Use SetAtIndex instead, or add a new operator.")
  virtual Eigen::VectorBlock<VectorX<T>> get_mutable_value() = 0;

  // VectorBase objects are neither copyable nor moveable.
  VectorBase(const VectorBase<T>& other) = delete;
  VectorBase& operator=(const VectorBase<T>& other) = delete;
  VectorBase(VectorBase<T>&& other) = delete;
  VectorBase& operator=(VectorBase<T>&& other) = delete;

 protected:
  VectorBase() {}
};

}  // namespace systems
}  // namespace drake
