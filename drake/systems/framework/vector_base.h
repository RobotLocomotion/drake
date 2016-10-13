#pragma once

#include <memory>

#include <Eigen/Dense>

#include "drake/common/drake_deprecated.h"
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

  virtual void SetZero() {
    int64_t sz = size();
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
  ///
  /// Implementations may override this default implementation with a more
  /// efficient approach, for instance if this vector is contiguous.
  /// Implementations should ensure this operation remains O(N) in the size of
  /// the value and allocates no memory.
  virtual VectorBase& PlusEqScaled(const T& scale, const VectorBase<T>& rhs) {
    if (rhs.size() != size()) {
      throw std::out_of_range("Addends must be the same size.");
    }
    for (int i = 0; i < size(); ++i) {
      SetAtIndex(i, GetAtIndex(i) + scale * rhs.GetAtIndex(i));
    }
    return *this;
  }

  /// Add in two scaled vectors, @p rhs1 and @p rh2 to this vector. All vectors
  /// must be the same size. This specialized function serves to minimize
  /// memory access.
  ///
  /// Implementations may override this default implementation with a more
  /// efficient approach, for instance if this vector is contiguous.
  /// Implementations should ensure this operation remains O(N) in the size of
  /// the value and allocates no memory.
  virtual VectorBase& PlusEqScaled(const T& scale1, const VectorBase<T>& rhs1,
                                   const T& scale2, const VectorBase<T>& rhs2) {
    if (rhs1.size() != size() || rhs2.size() != size()) {
      throw std::out_of_range("Addends must be the same size.");
    }
    for (int i = 0; i < size(); ++i) {
      SetAtIndex(i, GetAtIndex(i) + scale1 * rhs1.GetAtIndex(i) +
                        scale2 * rhs2.GetAtIndex(i));
    }
    return *this;
  }

  /// Add in three scaled vectors @p rhs1, @p rhs2, and @p rhs3 to this vector.
  /// All vectors must be the same size. This specialized function
  /// serves to minimize memory access.
  ///
  /// Implementations may override this default implementation with a more
  /// efficient approach, for instance if this vector is contiguous.
  /// Implementations should ensure this operation remains O(N) in the size of
  /// the value and allocates no memory.
  virtual VectorBase& PlusEqScaled(const T& scale1, const VectorBase<T>& rhs1,
                                   const T& scale2, const VectorBase<T>& rhs2,
                                   const T& scale3, const VectorBase<T>& rhs3) {
    if (rhs1.size() != size() || rhs2.size() != size() ||
        rhs3.size() != size()) {
      throw std::out_of_range("Addends must be the same size.");
    }
    for (int i = 0; i < size(); ++i) {
      SetAtIndex(i, GetAtIndex(i) + scale1 * rhs1.GetAtIndex(i) +
                        scale2 * rhs2.GetAtIndex(i) +
                        scale3 * rhs3.GetAtIndex(i));
    }
    return *this;
  }

  /// Adds in four scaled vectors @p rhs1, @p rhs2, @p rhs3, and @p rhs4 to this
  /// vector. All vectors must be the same size. This specialized function
  /// serves to minimize memory access.
  ///
  /// Implementations may override this default implementation with a more
  /// efficient approach, for instance if this vector is contiguous.
  /// Implementations should ensure this operation remains O(N) in the size of
  /// the value and allocates no memory.
  virtual VectorBase& PlusEqScaled(const T& scale1, const VectorBase<T>& rhs1,
                                   const T& scale2, const VectorBase<T>& rhs2,
                                   const T& scale3, const VectorBase<T>& rhs3,
                                   const T& scale4, const VectorBase<T>& rhs4) {
    if (rhs1.size() != size() || rhs2.size() != size() ||
        rhs3.size() != size() || rhs4.size() != size()) {
      throw std::out_of_range("Addends must be the same size.");
    }
    for (int i = 0; i < size(); ++i) {
      SetAtIndex(i, GetAtIndex(i) + scale1 * rhs1.GetAtIndex(i) +
                        scale2 * rhs2.GetAtIndex(i) +
                        scale3 * rhs3.GetAtIndex(i) +
                        scale4 * rhs4.GetAtIndex(i));
    }
    return *this;
  }

  /// Adds in five scaled vectors @p rhs1, @p rhs2, @p rhs3, @p rhs4, and
  /// @p rhs5 to this vector. All vectors must be the same size. This
  /// specialized function serves to minimize memory access.
  ///
  /// Implementations may override this default implementation with a more
  /// efficient approach, for instance if this vector is contiguous.
  /// Implementations should ensure this operation remains O(N) in the size of
  /// the value and allocates no memory.
  virtual VectorBase& PlusEqScaled(const T& scale1, const VectorBase<T>& rhs1,
                                   const T& scale2, const VectorBase<T>& rhs2,
                                   const T& scale3, const VectorBase<T>& rhs3,
                                   const T& scale4, const VectorBase<T>& rhs4,
                                   const T& scale5, const VectorBase<T>& rhs5) {
    if (rhs1.size() != size() || rhs2.size() != size() ||
        rhs3.size() != size() || rhs4.size() != size() ||
        rhs5.size() != size()) {
      throw std::out_of_range("Addends must be the same size.");
    }
    for (int i = 0; i < size(); ++i) {
      SetAtIndex(i,
                 GetAtIndex(i) + scale1 * rhs1.GetAtIndex(i) +
                     scale2 * rhs2.GetAtIndex(i) + scale3 * rhs3.GetAtIndex(i) +
                     scale4 * rhs4.GetAtIndex(i) + scale5 * rhs5.GetAtIndex(i));
    }
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
};

}  // namespace systems
}  // namespace drake
