#pragma once

#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include "drake/common/drake_throw.h"
#include "drake/common/dummy_value.h"
#include "drake/systems/framework/vector_base.h"

#include <Eigen/Dense>

namespace drake {
namespace systems {

/// BasicVector is a semantics-free wrapper around an Eigen vector that
/// satisfies VectorBase. Once constructed, its size is fixed.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
template <typename T>
class BasicVector : public VectorBase<T> {
 public:
  // BasicVector cannot be copied or moved; use Clone instead.  (We cannot
  // support copy or move because of the slicing problem, and also because
  // assignment of a BasicVector could change its size.)
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BasicVector)

  /// Initializes with the given @p size using the drake::dummy_value<T>, which
  /// is NaN when T = double.
  explicit BasicVector(int size)
      : values_(VectorX<T>::Constant(size, dummy_value<T>::get())) {}

  /// Constructs a BasicVector with the specified @p data.
  explicit BasicVector(const VectorX<T>& data) : values_(data) {}

  /// Constructs a BasicVector whose elements are the elements of @p data.
  static std::unique_ptr<BasicVector<T>> Make(
      const std::initializer_list<T>& data) {
    auto vec = std::make_unique<BasicVector<T>>(data.size());
    int i = 0;
    for (const T& datum : data) {
      vec->SetAtIndex(i++, datum);
    }
    return vec;
  }

  /// Constructs a BasicVector where each element is constructed using the
  /// placewise-corresponding member of @p args as the sole constructor
  /// argument.  For instance:
  ///   BasicVector<symbolic::Expression>::Make("x", "y", "z");
  template<typename... Fargs>
  static std::unique_ptr<BasicVector<T>> Make(Fargs&&... args) {
    auto data = std::make_unique<BasicVector<T>>(sizeof...(args));
    BasicVector<T>::MakeRecursive(data.get(), 0, args...);
    return std::move(data);
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

  /// Computes the infinity norm for this vector.
  T NormInf() const override {
    return values_.template lpNorm<Eigen::Infinity>();
  }

  /// Copies the entire vector to a new BasicVector, with the same concrete
  /// implementation type.
  ///
  /// Uses the Non-Virtual Interface idiom because smart pointers do not have
  /// type covariance.
  std::unique_ptr<BasicVector<T>> Clone() const {
    auto clone = std::unique_ptr<BasicVector<T>>(DoClone());
    clone->set_value(this->get_value());
    return clone;
  }

 protected:
  /// Returns a new BasicVector containing a copy of the entire vector.
  /// Caller must take ownership, and may rely on the NVI wrapper to initialize
  /// the clone elementwise.
  ///
  /// Subclasses of BasicVector must override DoClone to return their covariant
  /// type.
  virtual BasicVector<T>* DoClone() const {
    return new BasicVector<T>(this->size());
  }

  /// Sets @p data at @p index to an object of type T, which must have a
  /// single-argument constructor invoked via @p constructor_arg, and then
  /// recursively invokes itself on the next index with @p recursive args.
  /// Helper for BasicVector<T>::Make.
  template<typename F, typename... Fargs>
  static void MakeRecursive(BasicVector<T>* data, int index,
                            F constructor_arg, Fargs&&... recursive_args) {
    data->SetAtIndex(index++, T(constructor_arg));
    BasicVector<T>::MakeRecursive(data, index, recursive_args...);
  }

  /// Base case for the MakeRecursive template recursion.
  template<typename F, typename... Fargs>
  static void MakeRecursive(BasicVector<T>* data, int index,
                            F constructor_arg) {
    data->SetAtIndex(index++, T(constructor_arg));
  }

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
  // N.B. Do not add more member fields without considering the effect on
  // subclasses.  Derived class's Clone() methods currently assume that the
  // BasicVector(const VectorX<T>&) constructor is all that is needed.
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
