#pragma once

#include <initializer_list>
#include <ostream>
#include <stdexcept>
#include <utility>

#include <fmt/format.h>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_types.h"
#include "drake/common/nice_type_name.h"
#include "drake/common/unused.h"

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
/// @tparam_default_scalar
template <typename T>
class VectorBase {
 public:
  // VectorBase objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VectorBase)

  virtual ~VectorBase() {}

  /// Returns the number of elements in the vector.
  ///
  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.
  virtual int size() const = 0;

  /// Returns the element at the given index in the vector.
  /// @pre 0 <= `index` < size()
  T& operator[](int index) {
    DRAKE_ASSERT(index >= 0);
    return DoGetAtIndexUnchecked(index);
  }

  /// Returns the element at the given index in the vector.
  /// @pre 0 <= `index` < size()
  const T& operator[](int index) const {
    DRAKE_ASSERT(index >= 0);
    return DoGetAtIndexUnchecked(index);
  }

  /// Returns the element at the given index in the vector.
  /// @throws std::exception if the index is >= size() or negative.
  /// Consider operator[]() instead if bounds-checking is unwanted.
  const T& GetAtIndex(int index) const {
    if (index < 0) { this->ThrowOutOfRange(index); }
    return DoGetAtIndexChecked(index);
  }

  /// Returns the element at the given index in the vector.
  /// @throws std::exception if the index is >= size() or negative.
  /// Consider operator[]() instead if bounds-checking is unwanted.
  T& GetAtIndex(int index) {
    if (index < 0) { this->ThrowOutOfRange(index); }
    return DoGetAtIndexChecked(index);
  }

  /// Replaces the state at the given index with the value.
  /// @throws std::exception if the index is >= size().
  /// Consider operator[]() instead if bounds-checking is unwanted.
  void SetAtIndex(int index, const T& value) {
    GetAtIndex(index) = value;
  }

  /// Replaces the entire vector with the contents of @p value.
  /// @throws std::exception if @p value is not a column vector with size()
  /// rows.
  ///
  /// Implementations should ensure this operation is O(N) in the size of the
  /// value and allocates no memory.
  virtual void SetFrom(const VectorBase<T>& value) {
    const int n = value.size();
    if (n != size()) { this->ThrowMismatchedSize(n); }
    for (int i = 0; i < n; ++i) {
      (*this)[i] = value[i];
    }
  }

  /// Replaces the entire vector with the contents of @p value.
  /// @throws std::exception if @p value is not a column vector with size()
  /// rows.
  ///
  /// Implementations should ensure this operation is O(N) in the size of the
  /// value and allocates no memory.
  virtual void SetFromVector(const Eigen::Ref<const VectorX<T>>& value) {
    const int n = value.rows();
    if (n != size()) { this->ThrowMismatchedSize(n); }
    for (int i = 0; i < n; ++i) {
      (*this)[i] = value[i];
    }
  }

  /// Sets all elements of this vector to zero.
  virtual void SetZero() {
    const int n = size();
    for (int i = 0; i < n; ++i) {
      (*this)[i] = T(0.0);
    }
  }

  /// Copies this entire %VectorBase into a contiguous Eigen Vector.
  ///
  /// Implementations should ensure this operation is O(N) in the size of the
  /// value and allocates only the O(N) memory that it returns.
  virtual VectorX<T> CopyToVector() const {
    VectorX<T> vec(size());
    for (int i = 0; i < size(); ++i) {
      vec[i] = (*this)[i];
    }
    return vec;
  }

  /// Copies this entire %VectorBase into a pre-sized Eigen Vector.
  ///
  /// Implementations should ensure this operation is O(N) in the size of the
  /// value.
  /// @throws std::exception if `vec` is the wrong size.
  virtual void CopyToPreSizedVector(EigenPtr<VectorX<T>> vec) const {
    DRAKE_THROW_UNLESS(vec != nullptr);
    const int n = vec->rows();
    if (n != size()) { this->ThrowMismatchedSize(n); }
    for (int i = 0; i < n; ++i) {
      (*vec)[i] = (*this)[i];
    }
  }

  /// Adds a scaled version of this vector to Eigen vector @p vec.
  /// @throws std::exception if `vec` is the wrong size.
  ///
  /// Implementations should ensure this operation remains O(N) in the size of
  /// the value and allocates no memory.
  virtual void ScaleAndAddToVector(const T& scale,
                                   EigenPtr<VectorX<T>> vec) const {
    DRAKE_THROW_UNLESS(vec != nullptr);
    const int n = vec->rows();
    if (n != size()) { this->ThrowMismatchedSize(n); }
    for (int i = 0; i < n; ++i) {
      (*vec)[i] += scale * (*this)[i];
    }
  }

  /// Add in scaled vector @p rhs to this vector.
  /// @throws std::exception if @p rhs is a different size than this.
  VectorBase& PlusEqScaled(const T& scale, const VectorBase<T>& rhs) {
    return PlusEqScaled({{scale, rhs}});
  }

  /// Add in multiple scaled vectors to this vector.
  /// @throws std::exception if any rhs are a different size than this.
  VectorBase& PlusEqScaled(const std::initializer_list<
                           std::pair<T, const VectorBase<T>&>>& rhs_scale) {
    const int n = size();
    for (const auto& [scale, rhs] : rhs_scale) {
      unused(scale);
      const int rhs_n = rhs.size();
      if (rhs_n != n) { this->ThrowMismatchedSize(rhs_n); }
    }
    DoPlusEqScaled(rhs_scale);
    return *this;
  }

  /// Add in vector @p rhs to this vector.
  /// @throws std::exception if @p rhs is a different size than this.
  VectorBase& operator+=(const VectorBase<T>& rhs) {
    return PlusEqScaled(T(1), rhs);
  }

  /// Subtract in vector @p rhs to this vector.
  /// @throws std::exception if @p rhs is a different size than this.
  VectorBase& operator-=(const VectorBase<T>& rhs) {
    return PlusEqScaled(T(-1), rhs);
  }

  /// Get the bounds for the elements.
  /// If lower and upper are both empty size vectors, then there are no bounds.
  /// Otherwise, the bounds are (*lower)(i) <= GetAtIndex(i) <= (*upper)(i)
  /// The default output is no bounds.
  virtual void GetElementBounds(Eigen::VectorXd* lower,
                                Eigen::VectorXd* upper) const {
    lower->resize(0);
    upper->resize(0);
  }

 protected:
  VectorBase() {}

  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.  The index need not be validated when in release mode.
  virtual const T& DoGetAtIndexUnchecked(int index) const = 0;

  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.  The index need not be validated when in release mode.
  virtual T& DoGetAtIndexUnchecked(int index) = 0;

  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.  The index has already been checked for negative, but not size.
  /// Implementations must throw an exception when index >= size().
  virtual const T& DoGetAtIndexChecked(int index) const = 0;

  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.  The index has already been checked for negative, but not size.
  /// Implementations must throw an exception when index >= size().
  virtual T& DoGetAtIndexChecked(int index) = 0;

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
    const int n = size();
    for (int i = 0; i < n; ++i) {
      T value(0);
      for (const auto& [scale, rhs] : rhs_scale) {
        value += rhs[i] * scale;
      }
      (*this)[i] += value;
    }
  }

  [[noreturn]] void ThrowOutOfRange(int index) const {
    throw std::out_of_range(fmt::format(
        "Index {} is not within [0, {}) while accessing {}.",
        index, size(), NiceTypeName::Get(*this)));
  }

  [[noreturn]] void ThrowMismatchedSize(int other_size) const {
    throw std::out_of_range(fmt::format(
        "Operand vector size {} does not match this {} size {}",
        other_size, NiceTypeName::Get(*this), size()));
  }
};

/// Allows a VectorBase<T> to be streamed into a string as though it were a
/// RowVectorX<T>. This is useful for debugging purposes.
template <typename T>
std::ostream& operator<<(std::ostream& os, const VectorBase<T>& vec) {
  os << vec.CopyToVector().transpose();
  return os;
}

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::VectorBase)
