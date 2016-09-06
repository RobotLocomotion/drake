#pragma once

#include <cstdint>
#include <initializer_list>
#include <limits>
#include <memory>
#include <stdexcept>

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

  void set_value(const Eigen::Ref<const VectorX<T>>& value) override {
    if (value.rows() != values_.rows()) {
      throw std::out_of_range(
          "Cannot set a BasicVector of size " + std::to_string(values_.rows()) +
          " with a value of size " + std::to_string(value.rows()));
    }
    values_ = value;
  }

  int size() const override { return static_cast<int>(values_.rows()); }

  Eigen::VectorBlock<const VectorX<T>> get_value() const override {
    return values_.head(values_.rows());
  }

  Eigen::VectorBlock<VectorX<T>> get_mutable_value() override {
    return values_.head(values_.rows());
  }

  const T GetAtIndex(int index) const override {
    if (index >= size()) {
      throw std::out_of_range("Index " + std::to_string(index) +
                              " out of bounds for state vector of size " +
                              std::to_string(size()));
    }
    return values_[index];
  }

  void SetAtIndex(int index, const T& value) override {
    if (index >= size()) {
      throw std::out_of_range("Index " + std::to_string(index) +
                              " out of bounds for state vector of size " +
                              std::to_string(size()));
    }
    values_[index] = value;
  }

  void SetFromVector(const Eigen::Ref<const VectorX<T>>& value) override {
    set_value(value);
  }

  VectorX<T> CopyToVector() const override { return values_; }

  void ScaleAndAddToVector(const T& scale,
                           Eigen::Ref<VectorX<T>> vec) const override {
    if (vec.rows() != size()) {
      throw std::out_of_range("Addends must be the same length.");
    }
    vec += scale * values_;
  }

  BasicVector& PlusEqScaled(const T& scale,
                            const StateVector<T>& rhs) override {
    rhs.ScaleAndAddToVector(scale, values_);
    return *this;
  }

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
      : values_(other.values_) {}

  /// Returns a new BasicVector containing a copy of the entire vector.
  /// Caller must take ownership.
  ///
  /// Subclasses of BasicVector must override DoClone to return their covariant
  /// type.
  virtual BasicVector<T>* DoClone() const {
    return new BasicVector<T>(*this);
  }

 private:
  // The column vector of T values.
  VectorX<T> values_;
};

}  // namespace systems
}  // namespace drake
