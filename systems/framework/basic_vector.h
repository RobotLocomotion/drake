#pragma once

#include <cstdint>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>

#include <Eigen/Dense>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/common/dummy_value.h"
#include "drake/systems/framework/vector_base.h"

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

  /// Constructs an empty BasicVector.
  BasicVector();

  /// Initializes with the given @p size using the drake::dummy_value<T>, which
  /// is NaN when T = double.
  explicit BasicVector(int size)
      : values_(VectorX<T>::Constant(size, dummy_value<T>::get())) {}

  /// Constructs a BasicVector with the specified @p vec data.
  explicit BasicVector(VectorX<T> vec) : values_(std::move(vec)) {}

  /// Constructs a BasicVector whose elements are the elements of @p init.
  BasicVector(const std::initializer_list<T>& init)
      : BasicVector<T>(init.size()) {
    int i = 0;
    for (const T& datum : init) {
      this->SetAtIndex(i++, datum);
    }
  }

  /// Constructs a BasicVector whose elements are the elements of @p init.
  static std::unique_ptr<BasicVector<T>> Make(
      const std::initializer_list<T>& init) {
    return std::make_unique<BasicVector<T>>(init);
  }

  /// Constructs a BasicVector where each element is constructed using the
  /// placewise-corresponding member of @p args as the sole constructor
  /// argument.  For instance:
  ///   BasicVector<symbolic::Expression>::Make("x", "y", "z");
  template<typename... Fargs>
  static std::unique_ptr<BasicVector<T>> Make(Fargs&&... args) {
    auto data = std::make_unique<BasicVector<T>>(sizeof...(args));
    BasicVector<T>::MakeRecursive(data.get(), 0, args...);
    return data;
  }

  int size() const override { return static_cast<int>(values_.rows()); }

  /// Sets the vector to the given value. After a.set_value(b.get_value()), a
  /// must be identical to b.
  /// @throws std::out_of_range if the new value has different dimensions.
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

  /// Provides const access to the element storage.
  const VectorX<T>& values() const { return values_; }

  /// Provides mutable access to the element storage.
  VectorX<T>& values() { return values_; }

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
  // BasicVector(VectorX<T>) constructor is all that is needed.
};

// Workaround for https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57728 which
// should be moved back into the class definition once we no longer need to
// support GCC versions prior to 6.3.
template <typename T>
BasicVector<T>::BasicVector() = default;

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::BasicVector)
