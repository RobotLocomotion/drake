#pragma once

#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_throw.h"
#include "drake/systems/framework/vector_base.h"

#include <Eigen/Dense>

namespace drake {
namespace systems {

/// BasicVector is a semantics-free wrapper around an Eigen vector that
/// satisfies VectorBase. Once constructed, its size is fixed.
/// The BasicVector is initialized to the quiet_NaN of the Eigen scalar.
/// If numeric_limits is not specialized on the Eigen scalar, the BasicVector
/// will be initialized with the scalar's default constructor.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class BasicVector : public VectorBase<T> {
 public:
  explicit BasicVector(int size)
      : values_(VectorX<T>::Constant(
            size, std::numeric_limits<
                      typename Eigen::NumTraits<T>::Real>::quiet_NaN())) {}

  /// Constructs a BasicVector with the specified @p data.
  explicit BasicVector(const VectorX<T>& data) : values_(data) {}

  static std::unique_ptr<BasicVector<T>> Make(
      const std::initializer_list<T>& data) {
    auto vec = std::make_unique<BasicVector<T>>(data.size());
    int i = 0;
    for (const T& datum : data) {
      vec->SetAtIndex(i++, datum);
    }
    return vec;
  }

  int size() const override { return static_cast<int>(values_.rows()); }

  /// Sets the vector to the given value. After a.set_value(b.get_value()), a
  /// must be identical to b.
  /// Throws std::out_of_range if the new value has different dimensions.
  void set_value(const Eigen::Ref<const VectorX<T>>& value) {
    if (value.rows() != values_.rows()) {
      throw std::out_of_range(
          "Cannot set a BasicVector of size " + std::to_string(values_.rows()) +
          " with a value of size " + std::to_string(value.rows()));
    }
    values_ = value;
  }

  /// Returns the entire vector as a const Eigen::VectorBlock.
  Eigen::VectorBlock<const VectorX<T>> get_value() const {
    return values_.head(values_.rows());
  }

  /// Returns the entire vector as a mutable Eigen::VectorBlock, which allows
  /// mutation of the values, but does not allow resizing the vector itself.
  Eigen::VectorBlock<VectorX<T>> get_mutable_value() {
    return values_.head(values_.rows());
  }

  const T& GetAtIndex(int index) const override {
    DRAKE_THROW_UNLESS(index < size());
    return values_[index];
  }

  T& GetAtIndex(int index) override {
    DRAKE_THROW_UNLESS(index < size());
    return values_[index];
  }

  void SetFromVector(const Eigen::Ref<const VectorX<T>>& value) override {
    set_value(value);
  }

  VectorX<T> CopyToVector() const override { return values_; }

  void ScaleAndAddToVector(const T& scale,
                           Eigen::Ref<VectorX<T>> vec) const override {
    if (vec.rows() != size()) {
      throw std::out_of_range("Addends must be the same size.");
    }
    vec += scale * values_;
  }

  void SetZero() override { values_.setZero(); }

  /// Copies the entire vector to a new BasicVector, with the same concrete
  /// implementation type.
  ///
  /// Uses the Non-Virtual Interface idiom because smart pointers do not have
  /// type covariance.
  std::unique_ptr<BasicVector<T>> Clone() const {
    return std::unique_ptr<BasicVector<T>>(DoClone());
  }

  // Assignment of BasicVectors could change size, so we forbid it.
  BasicVector& operator=(const BasicVector& other) = delete;

  // BasicVector objects are not moveable.
  BasicVector(BasicVector&& other) = delete;
  BasicVector& operator=(BasicVector&& other) = delete;

 protected:
  explicit BasicVector(const BasicVector& other)
      : VectorBase<T>(), values_(other.values_) {}

  /// Returns a new BasicVector containing a copy of the entire vector.
  /// Caller must take ownership.
  ///
  /// Subclasses of BasicVector must override DoClone to return their covariant
  /// type.
  virtual BasicVector<T>* DoClone() const { return new BasicVector<T>(*this); }

 private:
  // Add in multiple scaled vectors to this vector. All vectors
  // must be the same size. This function overrides the default DoPlusEqScaled()
  // implementation toward maximizing speed. This implementation should be able
  // to leverage Eigen's fast scale and add functions in the case that rhs_scal
  // is also (i.e., in addition to 'this') a contiguous vector.
  void DoPlusEqScaled(
      const std::initializer_list<std::pair<T, const VectorBase<T>&>>& rhs_scal)
      override {
    for (const auto& operand : rhs_scal)
      operand.second.ScaleAndAddToVector(operand.first, values_);
  }

  // The column vector of T values.
  VectorX<T> values_;
};

// Allows a BasicVector<T> to be streamed into a string. This is useful for
// debugging purposes.
template <typename T>
std::ostream& operator<<(std::ostream& os, const BasicVector<T>& vec) {
  os << "[";

  Eigen::VectorBlock<const VectorX<T>> v = vec.get_value();
  for (int i = 0; i < v.size(); ++i) {
    if (i > 0)
       os << ", ";
    os << v[i];
  }

  os << "]";
  return os;
}

}  // namespace systems
}  // namespace drake
