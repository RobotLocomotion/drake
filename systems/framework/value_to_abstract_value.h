#pragma once

#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/value.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace internal {
/**
Implements Drake policy for taking an object of a concrete type and
storing it in an AbstractValue. The policy enforced here by these five
overloads is:
 1. Any Eigen type is copied into a `Value<BasicVector<T>>`.
 2. Any BasicVector or type derived from BasicVector is cloned into a
    `Value<BasicVector<T>>` that preserves the concrete type.
 3. Any AbstractValue object is simply cloned.
 4. A `char *` type is copied into a Value<std::string>.
 5. For any other type V
    - if V is copy-constructible it is copied into a `Value<V>`;
    - if V has a Clone() method that returns `unique_ptr<V>` it is cloned
      into a `Value<V>`;
    - if V has an accessible `Clone()` method that returns `unique_ptr<B>`,
      where B is a base class of T, it is cloned into a `Value<B>`;
    - otherwise, compilation fails with a `static_assert` message.

@tparam T The scalar type in use for vector objects.
*/
template <typename T>
class ValueToAbstractValue {
 public:
  // This signature is used for any Eigen vector type, but the argument is
  // copied to a BasicVector for the returned abstract value.
  static std::unique_ptr<AbstractValue> ToAbstract(
      const Eigen::Ref<const VectorX<T>>& vector) {
    return std::make_unique<Value<BasicVector<T>>>(vector);
  }

  // BasicVector must be special-cased so that we use a Value<BasicVector>
  // to store a copy of the given object, even if it is from a subclass of
  // BasicVector, and even if it has its own Clone() method. The actual type
  // is preserved regardless.
  static std::unique_ptr<AbstractValue> ToAbstract(
      const BasicVector<T>& vector) {
    return std::make_unique<Value<BasicVector<T>>>(vector.Clone());
  }

  // This signature is used for AbstractValue or Value<U> arguments.
  static std::unique_ptr<AbstractValue> ToAbstract(const AbstractValue& value) {
    return value.Clone();
  }

  // Special case char* to std::string to avoid ugly compilation messages
  // for this case, where the user's intent is obvious.
  static std::unique_ptr<AbstractValue> ToAbstract(const char* c_string) {
    return std::make_unique<Value<std::string>>(c_string);
  }

#ifdef DRAKE_DOXYGEN_CXX
  template <typename ValueType>
  static std::unique_ptr<AbstractValue> ToAbstract(const ValueType& value);
#else
  // Returns true iff ValueType is compatible with Eigen::Ref.
  template <typename ValueType>
  static constexpr bool is_eigen_refable() {
    return is_eigen_refable_helper<ValueType>(1);  // Any int will do here.
  }

  // Returns true iff ValueType has an accessible Clone() method that
  // returns a unique_ptr<ValueType> or one of ValueType's base classes.
  template <typename ValueType>
  static constexpr bool has_accessible_clone() {
    return has_accessible_clone_helper<ValueType>(1);  // Any int will do here.
  }

  // This method won't instantiate if any of the non-templatized signatures
  // above can be used (after possible conversions). If we allowed this to
  // instantiate it would be chosen instead of performing those conversions.
  template <typename ValueType,
            typename = std::enable_if_t<
                !(is_eigen_refable<ValueType>() ||
                  std::is_base_of<BasicVector<T>, ValueType>::value ||
                  std::is_base_of<AbstractValue, ValueType>::value)>>
  static std::unique_ptr<AbstractValue> ToAbstract(const ValueType& value) {
    static_assert(
        std::is_copy_constructible<ValueType>::value ||
            has_accessible_clone<ValueType>(),
        "ValueToAbstractValue(): value type must be copy constructible or "
        "have an accessible Clone() method that returns std::unique_ptr.");
    return ValueHelper(value, 1, 1);
  }
#endif

 private:
  // This overload is chosen for the int argument if the Ref type exists,
  // otherwise there is an SFINAE failure here.
  template <typename ValueType>
  static constexpr bool is_eigen_refable_helper(
      // That's a comma expression below, returning an int.
      decltype(std::declval<Eigen::Ref<ValueType>>(), int())) {
    return true;
  }

  // When the above method can't be instantiated, the int argument converts to
  // a char and this method is invoked instead.
  template <typename ValueType>
  static constexpr bool is_eigen_refable_helper(char) {
    return false;
  }

  template <typename ValueType>
  using CloneReturnType = std::remove_pointer_t<
      decltype(std::declval<const ValueType>().Clone().release())>;

  // This overload is chosen for the int argument if the Ref type exists,
  // otherwise there is an SFINAE failure here.
  template <typename ValueType, typename = CloneReturnType<ValueType>>
  static constexpr bool has_accessible_clone_helper(int) {
    return true;
  }

  // When the above method can't be instantiated, the int argument converts to
  // a char and this method is invoked instead.
  template <typename ValueType>
  static constexpr bool has_accessible_clone_helper(char) {
    return false;
  }

  // This is instantiated if ValueType has a copy constructor. If it is also
  // cloneable, this method still gets invoked; we prefer use of the the copy
  // constructor.
  template <
      typename ValueType,
      typename = std::enable_if_t<std::is_copy_constructible<ValueType>::value>>
  static std::unique_ptr<AbstractValue> ValueHelper(const ValueType& value, int,
                                                    int) {
    return std::make_unique<Value<ValueType>>(value);
  }

  // This is available if ValueType is cloneable, but is dispreferred if there
  // is also a copy constructor because the `int, int` args above are a better
  // match than the `int, ...` args here. The Clone() must return a unique_ptr
  // but the type could be a base type of ValueType rather than ValueType
  // itself. In that case we store the value using the base type, although
  // presumably the concrete type has been properly cloned.
  template <typename ValueType,
            typename ClonedValueType = CloneReturnType<ValueType>>
  static std::unique_ptr<AbstractValue> ValueHelper(const ValueType& value, int,
                                                    ...) {
    static_assert(
        std::is_base_of<ClonedValueType, ValueType>::value,
        "ValueToAbstractValue::ToAbstract(): accessible Clone() method must "
        "return ValueType or a base class of ValueType.");
    return std::make_unique<Value<ClonedValueType>>(value.Clone());
  }

  // This signature exists for non-copyable, non-cloneable ValueType just to
  // avoid spurious compilation errors. A static_assert above will have provided
  // a good message already; this method just needs to compile peacefully.
  template <typename ValueType>
  static std::unique_ptr<AbstractValue> ValueHelper(const ValueType&, ...) {
    DRAKE_UNREACHABLE();
  }
};

}  // namespace internal
}  // namespace systems
}  // namespace drake

