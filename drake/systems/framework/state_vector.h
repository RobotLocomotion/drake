#pragma once

#include <Eigen/Dense>

#include "drake/systems/framework/vector_interface.h"

namespace drake {
namespace systems {

/// StateVector is an abstract base class template for vector quantities within
/// the state of a System.  Both composite Systems (Diagrams) and leaf Systems
/// have state that satisfies StateVector.
///
/// @tparam T A mathematical type compatible with Eigen's Scalar.
template <typename T>
class StateVector {
 public:
  virtual ~StateVector() {}

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
  virtual void SetFromVector(const Eigen::Ref<const VectorX<T>>& value) = 0;

  /// Copies the entire state to a vector with no semantics.
  ///
  /// Implementations should ensure this operation is O(N) in the size of the
  /// value and allocates only the O(N) memory that it returns.
  virtual VectorX<T> CopyToVector() const = 0;

  /// Adds this vector to @p vec, which must be the same size.
  ///
  /// Implementations may override this default implementation with a more
  /// efficient approach, for instance if the vector is contiguous.
  /// Implementations should ensure this operation remains O(N) in the size of
  /// the value and allocates no memory.
  virtual void AddToVector(Eigen::Ref<VectorX<T>> vec) const {
    assert(vec.rows() == size());
    for (ptrdiff_t i = 0; i < size(); ++i) {
      vec[i] += GetAtIndex(i);
    }
  }

  /// Adds the vector @rhs to this vector. Both vectors must be the same size.
  ///
  /// Implementations may override this default implementation with a more
  /// efficient approach, for instance if the vector is contiguous.
  /// Implementations should ensure this operation remains O(N) in the size of
  /// the value and allocates no memory.
  virtual StateVector& operator+=(const StateVector<T>& rhs) {
    assert(size() == rhs.size());
    for (ptrdiff_t i = 0; i < size(); ++i) {
      SetAtIndex(i, GetAtIndex(i) + rhs.GetAtIndex(i));
    }
    return *this;
  }

 protected:
  StateVector() {}

 private:
  // StateVector objects are neither copyable nor moveable.
  StateVector(const StateVector& other) = delete;
  StateVector& operator=(const StateVector& other) = delete;
  StateVector(StateVector&& other) = delete;
  StateVector& operator=(StateVector&& other) = delete;
};

}  // namespace systems
}  // namespace drake
