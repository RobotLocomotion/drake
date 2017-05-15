#pragma once

#include <algorithm>
#include <memory>
#include <utility>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
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
/// be contiguous-in-memory. Contiguous subclasses should typically
/// inherit from BasicVector, not from VectorBase directly.
///
/// @tparam T Must be a Scalar compatible with Eigen.
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

  /// @returns `true` if and only if this vector can be expressed as an
  /// Eigen::VectorBlock in constant time.
  /// @see get_contiguous_vector(), get_mutable_contiguous_vector()
  bool is_contiguous() const {
    return is_segment_contiguous(0, size());
  }

  /// Returns `true` if the segment of `count` elements with first element at
  /// `start` in `this` vector has a contiguous-in-memory layout.
  /// This method throws a std::runtime_error if:
  ///  - `start` is out-of-bounds,
  ///  - `count` is either negative or larger than size()
  /// @see is_contiguous()
  bool is_segment_contiguous(int start, int count) const {
    VerifySegmentSizesOrThrow(start, count);
    return !!try_getting_contiguous_segment(start, count);
  }

  /// Returns the entire vector as an Eigen::VectorBlock referencing a
  /// contiguous block of memory, if possible.
  /// @throws std::runtime_error if is_contiguous() is `false`.
  /// @pre is_contiguous()
  Eigen::VectorBlock<const VectorX<T>> get_contiguous_vector() const {
    return get_contiguous_segment(0, size());
  }

  /// Returns the entire vector as a mutable Eigen::VectorBlock referencing a
  /// contiguous block of memory, if possible.
  /// @throws std::runtime_error if is_contiguous() is `false`.
  /// @pre is_contiguous()
  Eigen::VectorBlock<VectorX<T>> get_mutable_contiguous_vector() {
    return get_mutable_contiguous_segment(0, size());
  }

  /// Returns a segment of `count` elements with first element at `start` in
  /// `this` vector as an Eigen::VectorBlock referencing a contiguous block of
  /// memory, if possible.
  /// @throws std::runtime_error if is_segment_contiguous(start, count) is
  /// `false`.
  /// @pre is_segment_contiguous()
  Eigen::VectorBlock<const VectorX<T>> get_contiguous_segment(
      int start, int count) const {
    VerifySegmentSizesOrThrow(start, count);
    auto result = try_getting_contiguous_segment(start, count);
    DRAKE_THROW_UNLESS(is_segment_contiguous(start, count));
    return *result;
  }

  /// Returns a mutable segment of `count` elements with first element at
  /// `start` in `this` vector as a mutable Eigen::VectorBlock referencing a
  /// contiguous block of memory, if possible.
  /// @throws std::runtime_error if is_segment_contiguous(start, count) is
  /// `false`.
  /// @pre is_segment_contiguous()
  virtual Eigen::VectorBlock<VectorX<T>> get_mutable_contiguous_segment(
      int start, int count) {
    VerifySegmentSizesOrThrow(start, count);
    auto result = try_getting_mutable_contiguous_segment(start, count);
    DRAKE_THROW_UNLESS(is_segment_contiguous(start, count));
    return *result;
  }

  /// Replaces the entire vector with the contents of @p value. Throws
  /// std::runtime_error if @p value is not a column vector with size() rows.
  ///
  /// Implementations should ensure this operation is O(N) in the size of the
  /// value and allocates no memory.
  virtual void SetFrom(const VectorBase<T>& value) {
    DRAKE_THROW_UNLESS(value.size() == size());
    for (int i = 0; i < value.size(); ++i) {
      SetAtIndex(i, value.GetAtIndex((i)));
    }
  }

  /// Replaces the entire vector with the contents of @p value. Throws
  /// std::runtime_error if @p value is not a column vector with size() rows.
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
  ///
  /// @warning For cases when it is known that a contiguous segment can be
  /// retrieved, prefer using the faster variant get_contiguous_vector().
  ///
  /// @see get_contiguous_vector()
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

  /// Computes the infinity norm for this vector.
  ///
  /// You should override this method if possible with a more efficient
  /// approach that leverages structure; the default implementation performs
  /// element-by-element computations that are likely inefficient. If the
  /// vector is contiguous, for example, Eigen implementations should be far
  /// more efficient. Overriding implementations should
  /// ensure that this operation remains O(N) in the size of
  /// the value and allocates no memory.
  virtual T NormInf() const {
    using std::abs;
    using std::max;
    T norm(0);
    const int count = size();
    for (int i = 0; i < count; ++i) {
      T val = abs(GetAtIndex(i));
      norm = max(norm, val);
    }

    return norm;
  }

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

  /// Returns a drake::optional to a const Eigen::VectorBlock of `count`
  /// elements, with first element at `start` in `this` vector, if and only if:
  /// - the vector's values already lie in contiguous memory,
  /// - no memory allocation is required,
  /// - the block can be retrieved in O(1) time.
  /// Otherwise a drake::optional with no value should be returned.
  /// This is the NVI implementation to get_contiguous_segment() and therefore
  /// implementations are guaranteed to be called with valid `start` and `count`
  /// arguments.
  virtual optional<Eigen::VectorBlock<const VectorX<T>>>
  try_getting_contiguous_segment(int start, int count) const = 0;

  /// Returns a drake::optional to a mutable Eigen::VectorBlock of `count`
  /// elements, with first element at `start` in `this` vector, if and only if:
  /// - the vector's values already lie in contiguous memory,
  /// - no memory allocation is required,
  /// - the block can be retrieved in O(1) time.
  /// Otherwise a drake::optional with no value should be returned.
  /// This is the NVI implementation to get_mutable_contiguous_segment() and
  /// therefore implementations are guaranteed to be called with valid `start`
  /// and `count` arguments.
  virtual optional<Eigen::VectorBlock<VectorX<T>>>
  try_getting_mutable_contiguous_segment(int start, int count) = 0;

 private:
  // Helper method for the get_contiguous_segment() family of methods to verify
  // that:
  // - 0 <= start && start < this->size()
  // - 0 <= count && count <= this->size()
  // - (start + count) <= this->size()
  // Or a std::runtime_error is thrown if the above conditions are not met.
  void VerifySegmentSizesOrThrow(int start, int count) const {
    const int sz = size();
    DRAKE_THROW_UNLESS(0 <= start && start < sz);
    DRAKE_THROW_UNLESS(0 <= count && count <= sz);
    DRAKE_THROW_UNLESS((start + count) <= sz);
  }
};

}  // namespace systems
}  // namespace drake
