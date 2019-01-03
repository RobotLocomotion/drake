#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"
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
  // VectorBase objects are neither copyable nor moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VectorBase)

  virtual ~VectorBase() {}

  /// Returns the number of elements in the vector.
  virtual int size() const { return lower_bound_.rows(); }

  /// Returns the element at the given index in the vector.
  /// @throws std::runtime_error if the index is >= size().
  ///
  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.
  virtual const T& GetAtIndex(int index) const = 0;

  /// Returns the element at the given index in the vector.
  /// @throws std::runtime_error if the index is >= size().
  ///
  /// Implementations should ensure this operation is O(1) and allocates no
  /// memory.
  virtual T& GetAtIndex(int index) = 0;

  T& operator[](std::size_t idx) { return GetAtIndex(idx); }
  const T& operator[](std::size_t idx) const { return GetAtIndex(idx); }

  /// Replaces the state at the given index with the value.
  /// @throws std::runtime_error if the index is >= size().
  void SetAtIndex(int index, const T& value) {
    GetAtIndex(index) = value;
  }

  /// Replaces the entire vector with the contents of @p value.
  /// @throws std::runtime_error if @p value is not a column vector with size()
  /// rows.
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

  /// The lower bound of vector value.
  const Eigen::VectorXd& lower_bound() const { return lower_bound_; }

  /// The upper bound of the vector value .
  const Eigen::VectorXd& upper_bound() const { return upper_bound_; }

  void SetLowerBound(const Eigen::Ref<const Eigen::VectorXd>& lower_bound) {
    DRAKE_ASSERT(lower_bound.size() == this->size());
    DRAKE_ASSERT((lower_bound.array() <= upper_bound_.array()).all());
    lower_bound_ = lower_bound;
  }

  void SetUpperBound(const Eigen::Ref<const Eigen::VectorXd>& upper_bound) {
    DRAKE_ASSERT(upper_bound.size() == this->size());
    DRAKE_ASSERT((lower_bound_.array() <= upper_bound.array()).all());
    upper_bound_ = upper_bound;
  }

  void SetBounds(const Eigen::Ref<const Eigen::VectorXd>& lower_bound,
                 const Eigen::Ref<const Eigen::VectorXd>& upper_bound) {
    DRAKE_ASSERT(lower_bound.size() == this->size());
    DRAKE_ASSERT(upper_bound.size() == this->size());
    DRAKE_ASSERT((lower_bound.array() <= upper_bound.array()).all());
    lower_bound_ = lower_bound;
    upper_bound_ = upper_bound;
  }

  /// Set the lower and upper bounds at the given index
  void SetBounds(int index, double lower_bound, double upper_bound) {
    DRAKE_ASSERT(index < size());
    DRAKE_ASSERT(lower_bound <= upper_bound);
    lower_bound_(index) = lower_bound;
    upper_bound_(index) = upper_bound;
  }

  /// Set the lower bound at the given index.
  void SetLowerBound(int index, double lower_bound) {
    DRAKE_ASSERT(index < size());
    DRAKE_ASSERT(lower_bound <= upper_bound_(index));
    lower_bound_(index) = lower_bound;
  }

  /// Set the upper bound at the given index.
  void SetUpperBound(int index, double upper_bound) {
    DRAKE_ASSERT(index < size());
    DRAKE_ASSERT(lower_bound_(index) <= upper_bound);
    upper_bound_(index) = upper_bound;
  }

 protected:
  VectorBase() : lower_bound_{0}, upper_bound_{0} {}

  /// Constructs a VectorBase with the given size. The lower and upper bounds
  /// are set to -inf and inf respectively.
  explicit VectorBase(int size)
      : lower_bound_{Eigen::VectorXd::Constant(
            size, -std::numeric_limits<double>::infinity())},
        upper_bound_{Eigen::VectorXd::Constant(
            size, std::numeric_limits<double>::infinity())} {}

  /// Constructs a VectorBase with the given lower and upper bounds.
  VectorBase(const Eigen::Ref<const Eigen::VectorXd>& lower_bound,
             const Eigen::Ref<const Eigen::VectorXd>& upper_bound)
      : lower_bound_{lower_bound}, upper_bound_{upper_bound} {
    DRAKE_ASSERT(lower_bound_.size() == upper_bound_.size());
    DRAKE_ASSERT((lower_bound_.array() <= upper_bound_.array()).all());
  }

  /// Adds in multiple scaled vectors to this vector. All vectors
  /// are guaranteed to be the same size.
  ///
  /// You should override this method if possible with a more efficient
  /// approach that leverages structure; the default implementation performs
  /// element-by-element computations that are likely inefficient, but even
  /// this implementation minimizes memory accesses for efficiency. If the
  /// vector is contiguous, for example, implementations that leverage SIMD
  /// operations should be far more efficient. Overriding implementations
  /// should
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

  /// Initialize the lower and upper bounds to -inf and inf respectively.
  void InitializeBounds() {
    const double kInf = std::numeric_limits<double>::infinity();
    lower_bound_ = Eigen::VectorXd::Constant(size(), -kInf);
    upper_bound_ = Eigen::VectorXd::Constant(size(), kInf);
  }

 private:
  Eigen::VectorXd lower_bound_;
  Eigen::VectorXd upper_bound_;
};

// Allows a VectorBase<T> to be streamed into a string. This is useful for
// debugging purposes.
template <typename T>
std::ostream& operator<<(std::ostream& os, const VectorBase<T>& vec) {
  os << "[";

  for (int i = 0; i < vec.size(); ++i) {
    if (i > 0)
      os << ", ";
    os << vec.GetAtIndex(i);
  }

  os << "]";
  return os;
}

}  // namespace systems
}  // namespace drake
