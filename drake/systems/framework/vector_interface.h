#pragma once

#include <Eigen/Dense>

namespace drake {
namespace systems {

/// VectorInterface is a pure abstract interface that real-valued signals
/// between Systems must satisfy. Classes that inherit from VectorInterface
/// will typically provide names for the elements of the vector, and may also
/// provide other computations for the convenience of Systems handling the
/// signal. The vector is always a column vector.
template <typename ScalarType>
class VectorInterface {
 public:
  /// After a.Initialize(b.value()), a must be identical to b.
  /// Throws a runtime error if value has incorrect dimensions.
  virtual void Initialize(
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
