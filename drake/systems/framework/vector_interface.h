#pragma once

#include <Eigen/Dense>

namespace drake {
namespace systems {

template <typename ScalarType>
using VectorX = Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>;

/// VectorInterface is a pure abstract interface that real-valued signals
/// between Systems must satisfy. Classes that inherit from VectorInterface
/// will typically provide names for the elements of the vector, and may also
/// provide other computations for the convenience of Systems handling the
/// signal. The vector is always a column vector.
///
/// @tparam ScalarType Must be a Scalar compatible with Eigen.
template <typename ScalarType>
class VectorInterface {
 public:
  virtual ~VectorInterface() {}

  // VectorInterfaces are neither copyable nor moveable.
  VectorInterface(const VectorInterface<ScalarType>& other) = delete;
  VectorInterface& operator=(const VectorInterface<ScalarType>& other) = delete;
  VectorInterface(VectorInterface<ScalarType>&& other) = delete;
  VectorInterface& operator=(VectorInterface<ScalarType>&& other) = delete;

  /// After a.set_value(b.get_value()), a must be identical to b.
  /// May throw std::runtime_error if the new value has different dimensions
  /// than expected by the concrete class implementing VectorInterface.
  virtual void set_value(const VectorX<ScalarType>& value) = 0;

  /// Returns a column vector containing the entire current value of the signal.
  virtual const VectorX<ScalarType>& get_value() const = 0;

  /// Returns a reference that allows mutation of the values in this vector, but
  /// not resize of the vector itself.
  virtual Eigen::VectorBlock<VectorX<ScalarType>> get_mutable_value() = 0;

 protected:
  VectorInterface() {}
};

}  // namespace systems
}  // namespace drake
