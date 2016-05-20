#pragma once

#include <Eigen/Dense>

#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// StateVectorInterface is an interface template for vector quantities within
/// the state of a System.  Both composite Systems (Diagrams) and Systems that
/// have no component systems have state that satisfies StateVectorInterface.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class StateVectorInterface {
 public:
  virtual ~StateVectorInterface() {}

  /// Returns the number of elements in the vector.
  ///
  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.
  virtual ptrdiff_t size() const = 0;

  /// Returns the element at the given index in the vector. Throws
  /// std::out_of_range if the index is >= size().
  ///
  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.
  virtual const T GetAtIndex(ptrdiff_t index) const = 0;

  /// Replaces the state at the given index with the value. Throws
  /// std::out_of_range if the index is >= size().
  ///
  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.
  virtual void SetAtIndex(ptrdiff_t index, const T& value) = 0;

  /// Replaces the entire state with the contents of value. Throws
  /// std::out_of_range if value is not a column vector with size() rows.
  ///
  /// Implementations should ensure this operation is O(N) in the size of the
  /// value and allocates no memory.
  virtual void SetFromVector(const Eigen::Ref<VectorX<T>>& value) = 0;

 protected:
  StateVectorInterface() {}

 private:
  // StateVectorInterface objects are neither copyable nor moveable.
  StateVectorInterface(const StateVectorInterface& other) = delete;
  StateVectorInterface& operator=(const StateVectorInterface& other) = delete;
  StateVectorInterface(StateVectorInterface&& other) = delete;
  StateVectorInterface& operator=(StateVectorInterface&& other) = delete;
};

}  // namespace systems
}  // namespace drake
