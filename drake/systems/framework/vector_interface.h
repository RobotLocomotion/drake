#pragma once

#include <Eigen/Dense>

namespace drake {
namespace systems {

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
  VectorInterface() {}
  virtual ~VectorInterface() {}

  // VectorInterfaces are neither copyable nor moveable.
  VectorInterface(const VectorInterface<ScalarType>& other) = delete;
  VectorInterface& operator=(const VectorInterface<ScalarType>& other) = delete;
  VectorInterface(VectorInterface<ScalarType>&& other) = delete;
  VectorInterface& operator=(VectorInterface<ScalarType>&& other) = delete;

  /// After a.set_value(b.get_value()), a must be identical to b.
  /// May throw a runtime error if the new value has different dimensions
  /// than expected by the concrete class implementing VectorInterface.
  virtual void set_value(
      const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>& value) = 0;

  /// Returns a real-valued column vector representing the entire state of
  /// this signal.
  virtual const Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>& get_value()
      const = 0;

  /// Returns a pointer that allows mutation of the state of this signal.
  virtual Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>* get_mutable_value() = 0;
};

}  // namespace systems
}  // namespace drake
