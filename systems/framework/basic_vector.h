#pragma once

#include <initializer_list>
#include <memory>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/common/dummy_value.h"
#include "drake/common/eigen_types.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {

/// BasicVector is a semantics-free wrapper around an Eigen vector that
/// satisfies VectorBase. Once constructed, its size is fixed.
///
/// @tparam_default_scalar
template <typename T>
class BasicVector : public VectorBase<T> {
 public:
  // BasicVector cannot be copied or moved; use Clone instead.  (We cannot
  // support copy or move because of the slicing problem, and also because
  // assignment of a BasicVector could change its size.)
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BasicVector)

  /// Constructs an empty BasicVector.
  BasicVector() = default;

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
      (*this)[i++] = datum;
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

  int size() const final { return static_cast<int>(values_.rows()); }

  /// Sets the vector to the given value. After a.set_value(b.get_value()), a
  /// must be identical to b.
  /// @throws std::exception if the new value has different dimensions.
  void set_value(const Eigen::Ref<const VectorX<T>>& value) {
    const int n = value.rows();
    if (n != size()) { this->ThrowMismatchedSize(n); }
    values_ = value;
  }

  /// Returns a const reference to the contained `VectorX<T>`. This is the
  /// preferred method for examining a BasicVector's value.
  const VectorX<T>& value() const { return values_; }

  /// Returns the entire vector as a mutable Eigen::VectorBlock, which allows
  /// mutation of the values, but does not allow `resize()` to be invoked on
  /// the returned object.
  Eigen::VectorBlock<VectorX<T>> get_mutable_value() {
    return values_.head(values_.rows());
  }

  void SetFromVector(const Eigen::Ref<const VectorX<T>>& value) final {
    set_value(value);
  }

  VectorX<T> CopyToVector() const final { return values_; }

  void ScaleAndAddToVector(const T& scale,
                           EigenPtr<VectorX<T>> vec) const final {
    DRAKE_THROW_UNLESS(vec != nullptr);
    const int n = vec->rows();
    if (n != size()) { this->ThrowMismatchedSize(n); }
    *vec += scale * values_;
  }

  void SetZero() final { values_.setZero(); }

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

  // TODO(sherm1) Consider deprecating this.
  /// (Don't use this in new code) Returns the entire vector as a const
  /// Eigen::VectorBlock. Prefer `value()` which returns direct access to the
  /// underlying VectorX rather than wrapping it in a VectorBlock.
  Eigen::VectorBlock<const VectorX<T>> get_value() const {
    return values_.head(values_.rows());
  }

 protected:
  const T& DoGetAtIndexUnchecked(int index) const final {
    DRAKE_ASSERT(index < size());
    return values_[index];
  }

  T& DoGetAtIndexUnchecked(int index) final {
    DRAKE_ASSERT(index < size());
    return values_[index];
  }

  const T& DoGetAtIndexChecked(int index) const final {
    if (index >= size()) { this->ThrowOutOfRange(index); }
    return values_[index];
  }

  T& DoGetAtIndexChecked(int index) final {
    if (index >= size()) { this->ThrowOutOfRange(index); }
    return values_[index];
  }

  /// Returns a new BasicVector containing a copy of the entire vector.
  /// Caller must take ownership, and may rely on the NVI wrapper to initialize
  /// the clone elementwise.
  ///
  /// Subclasses of BasicVector must override DoClone to return their covariant
  /// type.
  [[nodiscard]] virtual BasicVector<T>* DoClone() const {
    return new BasicVector<T>(this->size());
  }

  /// Sets @p data at @p index to an object of type T, which must have a
  /// single-argument constructor invoked via @p constructor_arg, and then
  /// recursively invokes itself on the next index with @p recursive args.
  /// Helper for BasicVector<T>::Make.
  template<typename F, typename... Fargs>
  static void MakeRecursive(BasicVector<T>* data, int index,
                            F constructor_arg, Fargs&&... recursive_args) {
    (*data)[index++] = T(constructor_arg);
    BasicVector<T>::MakeRecursive(data, index, recursive_args...);
  }

  /// Base case for the MakeRecursive template recursion.
  template<typename F, typename... Fargs>
  static void MakeRecursive(BasicVector<T>* data, int index,
                            F constructor_arg) {
    (*data)[index++] = T(constructor_arg);
  }

  // TODO(sherm1) Deprecate this method.
  /// Provides const access to the element storage. Prefer the synonymous
  /// public `value()` method -- this protected method remains for backwards
  /// compatibility in derived classes.
  const VectorX<T>& values() const { return values_; }

  /// (Advanced) Provides mutable access to the element storage. Be careful
  /// not to resize the storage unless you really know what you're doing.
  VectorX<T>& values() { return values_; }

 private:
  // Add in multiple scaled vectors to this vector. All vectors
  // must be the same size. This function overrides the default DoPlusEqScaled()
  // implementation toward maximizing speed. This implementation should be able
  // to leverage Eigen's fast scale and add functions in the case that rhs_scal
  // is also (i.e., in addition to 'this') a contiguous vector.
  void DoPlusEqScaled(
      const std::initializer_list<std::pair<T, const VectorBase<T>&>>& rhs_scal)
      final {
    for (const auto& operand : rhs_scal)
      operand.second.ScaleAndAddToVector(operand.first, &values_);
  }

  // The column vector of T values.
  VectorX<T> values_;
  // N.B. Do not add more member fields without considering the effect on
  // subclasses.  Derived class's Clone() methods currently assume that the
  // BasicVector(VectorX<T>) constructor is all that is needed.
};

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::BasicVector)
