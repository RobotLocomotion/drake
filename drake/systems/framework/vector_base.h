#pragma once

#include <memory>
#include <utility>

#include <Eigen/Dense>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {

/// VectorBase is an abstract base class that real-valued signals
/// between Systems and real-valued System state vectors must implement.
/// Classes that inherit from VectorBase will typically provide names
/// for the elements of the vector, and may also provide other
/// computations for the convenience of Systems handling the
/// signal. The vector is always a column vector. It may or may not
/// be contiguous in memory. Contiguous subclasses should typically
/// inherit from BasicVector, not from VectorBase directly.
///
/// @tparam T Must be a Scalar compatible with Eigen.
template <typename T>
class VectorBase {
 public:
  virtual ~VectorBase() {}

  /// Returns the number of elements in the vector.
  ///
  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.
  virtual int size() const = 0;

  /// Returns the element at the given index in the vector. Throws
  /// std::runtime_error if the index is >= size().
  ///
  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.
  virtual const T& GetAtIndex(int index) const = 0;

  /// Returns the element at the given index in the vector. Throws
  /// std::runtime_error if the index is >= size().
  ///
  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.
  virtual T& GetAtIndex(int index) = 0;

  T& operator[](std::size_t idx) { return GetAtIndex(idx); }
  const T& operator[](std::size_t idx) const { return GetAtIndex(idx); }

  /// Replaces the state at the given index with the value. Throws
  /// std::runtime_error if the index is >= size().
  void SetAtIndex(int index, const T& value) {
    GetAtIndex(index) = value;
  }

  /// Replaces the entire state with the contents of value. Throws
  /// std::runtime_error if value is not a column vector with size() rows.
  ///
  /// Implementations should ensure this operation is O(N) in the size of the
  /// value and allocates no memory.
  virtual void SetFromVector(const Eigen::Ref<const VectorX<T>>& value) {
    DRAKE_THROW_UNLESS(value.rows() == size());
    for (int i = 0; i < value.rows(); ++i) {
      SetAtIndex(i, value[i]);
    }
  }

  virtual void SetZero() {
    const int sz = size();
    for (int i = 0; i < sz; ++i) {
      SetAtIndex(i, T(0));
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

  /// Adds a scaled version of this vector to Eigen vector @p vec, which
  /// must be the same size.
  ///
  /// Implementations may override this default implementation with a more
  /// efficient approach, for instance if this vector is contiguous.
  /// Implementations should ensure this operation remains O(N) in the size of
  /// the value and allocates no memory.
  virtual void ScaleAndAddToVector(const T& scale,
                                   Eigen::Ref<VectorX<T>> vec) const {
    if (vec.rows() != size()) {
      throw std::out_of_range("Addends must be the same size.");
    }
    for (int i = 0; i < size(); ++i) vec[i] += scale * GetAtIndex(i);
  }

  /// Add in scaled vector @p rhs to this vector. Both vectors must
  /// be the same size.
  VectorBase& PlusEqScaled(const T& scale, const VectorBase<T>& rhs) {
    return PlusEqScaled({{scale, rhs}});
  }

  /// Add in multiple scaled vectors to this vector. All vectors
  /// must be the same size.
  VectorBase& PlusEqScaled(const std::initializer_list<
                           std::pair<T, const VectorBase<T>&>>& rhs_scale) {
    for (const auto& operand : rhs_scale) {
      if (operand.second.size() != size())
        throw std::out_of_range("Addends must be the same size.");
    }

    DoPlusEqScaled(rhs_scale);

    return *this;
  }

  /// Add in vector @p rhs to this vector.
  VectorBase& operator+=(const VectorBase<T>& rhs) {
    return PlusEqScaled(T(1), rhs);
  }

  /// Subtract in vector @p rhs to this vector.
  VectorBase& operator-=(const VectorBase<T>& rhs) {
    return PlusEqScaled(T(-1), rhs);
  }

  // VectorBase objects are neither copyable nor moveable.
  VectorBase(const VectorBase<T>& other) = delete;
  VectorBase& operator=(const VectorBase<T>& other) = delete;
  VectorBase(VectorBase<T>&& other) = delete;
  VectorBase& operator=(VectorBase<T>&& other) = delete;

 protected:
  VectorBase() {}

  /// Adds in multiple scaled vectors to this vector. All vectors
  /// are guaranteed to be the same size.
  ///
  /// You should override this method if possible with a more efficient
  /// approach that leverages structure; the default implementation performs
  /// element-by-element computations that are likely inefficient, but even
  /// this implementation minimizes memory accesses for efficiency. If the
  /// vector is contiguous, for example, implementations that leverage SIMD
  /// operations should be far more efficient. Overriding implementations should
  /// ensure that this operation remains O(N) in the size of
  /// the value and allocates no memory.
  virtual void DoPlusEqScaled(const std::initializer_list<
                              std::pair<T, const VectorBase<T>&>>& rhs_scale) {
    const int sz = size();
    for (int i = 0; i < sz; ++i) {
      T value(0);
      for (const auto& operand : rhs_scale)
        value += operand.second.GetAtIndex(i) * operand.first;
      SetAtIndex(i, GetAtIndex(i) + value);
    }
  }
};

}  // namespace systems
}  // namespace drake
