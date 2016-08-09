#pragma once

#include <cstdint>
#include <stdexcept>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
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
  virtual int size() const = 0;

  /// Returns the element at the given index in the vector. Throws
  /// std::out_of_range if the index is >= size().
  ///
  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.
  virtual const T GetAtIndex(int index) const = 0;

  /// Replaces the state at the given index with the value. Throws
  /// std::out_of_range if the index is >= size().
  ///
  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.
  virtual void SetAtIndex(int index, const T& value) = 0;

  /// Replaces the entire state with the contents of value. Throws
  /// std::out_of_range if value is not a column vector with size() rows.
  ///
  /// Implementations should ensure this operation is O(N) in the size of the
  /// value and allocates no memory.
  virtual void SetFromVector(const Eigen::Ref<const VectorX<T>>& value) {
    for (int i = 0; i < value.rows(); ++i) {
      SetAtIndex(i, value[i]);
    }
  }

  /// Copies the entire state to a vector with no semantics.
  ///
  /// Implementations should ensure this operation is O(N) in the size of the
  /// value and allocates only the O(N) memory that it returns.
  virtual VectorX<T> CopyToVector() const {
    VectorX<T> vec(size());
    for (int i = 0; i < size(); ++i) {
      vec[i] = GetAtIndex(i);
    }
    return vec;
  }

  /// Adds a scaled version of this state vector to Eigen vector @p vec, which
  /// must be the same size.
  ///
  /// Implementations may override this default implementation with a more
  /// efficient approach, for instance if this vector is contiguous.
  /// Implementations should ensure this operation remains O(N) in the size of
  /// the value and allocates no memory.
  virtual void ScaleAndAddToVector(const T& scale,
                                   Eigen::Ref<VectorX<T>> vec) const {
    if (vec.rows() != size()) {
      throw std::out_of_range("Addends must be the same length.");
    }
    for (int i = 0; i < size(); ++i)
      vec[i] += scale * GetAtIndex(i);
  }

  /// Add in scaled state vector @p rhs to this state vector. Both vectors must
  /// be the same size.
  ///
  /// Implementations may override this default implementation with a more
  /// efficient approach, for instance if this vector is contiguous.
  /// Implementations should ensure this operation remains O(N) in the size of
  /// the value and allocates no memory.
  virtual StateVector& PlusEqScaled(const T& scale, const StateVector<T>& rhs) {
    if (rhs.size() != size()) {
      throw std::out_of_range("Addends must be the same length.");
    }
    for (int i = 0; i < size(); ++i)
      SetAtIndex(i, GetAtIndex(i) + scale * rhs.GetAtIndex(i));
    return *this;
  }

  /// Add in state vector @p rhs to this state vector.
  StateVector& operator+=(const StateVector<T>& rhs) {
    return PlusEqScaled(T(1), rhs);
  }

  /// Subtract in state vector @p rhs to this state vector.
  StateVector& operator-=(const StateVector<T>& rhs) {
      return PlusEqScaled(T(-1), rhs);
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
