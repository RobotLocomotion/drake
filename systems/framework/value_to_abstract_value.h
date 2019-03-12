#pragma once

#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/unused.h"
#include "drake/common/value.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace internal {

// is_eigen_refable() returns a constexpr bool saying whether its template
// argument is any Eigen expression acceptable to Eigen::Ref. That includes
// all the Eigen things we care about.

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

// Returns true iff ValueType is compatible with Eigen::Ref.
template <typename ValueType>
static constexpr bool is_eigen_refable() {
  return is_eigen_refable_helper<ValueType>(1);  // Any int will do here.
}

/** Implements Drake policy for taking a concrete value object and storing it in
a Drake abstract object (for example, an abstract-valued port) as a type-erased
AbstractValue. We call this "AbstractPolicy" to distinguish it from the
"VectorPolicy" implemented by ValueToVectorValue.

<h4>Usage</h4>

This class can be used in conjunction with ValueToVectorValue to store an
arbitrary value object into a Drake abstract or vector object, using the
appropriate policy. Here is an example:
```
  std::unique_ptr<AbstractValue> abstract_value =
    is_vector_port
        ? internal::ValueToVectorValue<T>::ToAbstract(__func__, value)
        : internal::ValueToAbstractValue::ToAbstract(value);
```
Note that for this to work _both_ ToAbstract() methods must compile
successfully, even though the VectorPolicy is much more restrictive. Thus
the VectorPolicy must issue runtime errors for values that are unacceptable.

@see ValueToVectorValue

<h4>AbstractPolicy</h4>

 1. Any given AbstractValue object is simply cloned.
 2. A `char *` type is copied into a Value<std::string>.
 3. Any Eigen expression type is simplified using `.eval()`, and then stored
    in a Value<E> where E is the type returned by `.eval()` (without const).
 4. For any other type V
    - if V is copy-constructible it is copied into a `Value<V>`;
    - if V has an accessible Clone() method that returns `unique_ptr<V>` it is
      cloned into a `Value<V>`;
    - if V has an accessible `Clone()` method that returns `unique_ptr<B>`,
      where B is a base class of V, then V is cloned into a `Value<B>`;
    - otherwise, compilation fails with a `static_assert` message.

@warning Eigen expressions typically don't have simple Vector types. That
doesn't matter under the VectorPolicy. However, under the AbstractPolicy you
will get Value<E> where E is the type of V.eval(). If you need to know E's
type, use `ValueToAbstractValue::NiceEigenType<V>`. Better, put your expression
V into an Eigen vector of known type before storing it in an AbstractValue. */
class ValueToAbstractValue {
 public:
  // Signature (1): used for AbstractValue or Value<U> arguments.
  static std::unique_ptr<AbstractValue> ToAbstract(const AbstractValue& value) {
    return value.Clone();
  }

  // Signature (2): special case char* to std::string to avoid ugly compilation
  // messages for this case, where the user's intent is obvious.
  static std::unique_ptr<AbstractValue> ToAbstract(const char* c_string) {
    return std::make_unique<Value<std::string>>(c_string);
  }

  // This is the storage type we'll use for an Eigen vector type.
  template <typename ValueType,
            typename = std::enable_if_t<is_eigen_refable<ValueType>()>>
  using NiceEigenType = std::remove_const_t<
      std::remove_reference_t<decltype(std::declval<ValueType>().eval())>>;

  // Signature (3): special case any Eigen vector expression so that we
  // can invoke eval() on it and store it as the simpler result type. If
  // you need to know how it was stored, use NiceEigenType above.

  // Note that we're assuming that MatrixBase<Derived> will instantiate under
  // the same conditions as is_eigen_refable() returns true.
  // TODO(sherm1) Figure out some way to make this explicitly dependent on
  // is_eigen_refable<ValueType>().
  template <typename Derived>
  static std::unique_ptr<AbstractValue> ToAbstract(
      const Eigen::MatrixBase<Derived>& eigen_expr) {
    // Can't use NiceEigenType here because we don't have the original
    // ValueType.
    using TypeAfterEval = std::remove_const_t<
        std::remove_reference_t<decltype(eigen_expr.eval())>>;
    return std::make_unique<Value<TypeAfterEval>>(eigen_expr.eval());
  }

  // Returns true iff ValueType has an accessible Clone() method that
  // returns a unique_ptr<ValueType> or one of ValueType's base classes.
  template <typename ValueType>
  static constexpr bool has_accessible_clone() {
    return has_accessible_clone_helper<ValueType>(1);  // Any int will do here.
  }

  // Signature (4): This signature won't instantiate if signature (1) or (3)
  // can be used (after possible conversions). If we allowed this to instantiate
  // it would be chosen instead of performing those conversions. Note that for
  // the AbstractPolicy, BasicVector ValueTypes are handled with this generic
  // method rather than by a specialized method as for VectorPolicy.
  template <typename ValueType,
            typename = std::enable_if_t<
                !(std::is_base_of<AbstractValue, ValueType>::value ||
                  is_eigen_refable<ValueType>())>>
  static std::unique_ptr<AbstractValue> ToAbstract(const ValueType& value) {
    static_assert(
        std::is_copy_constructible<ValueType>::value ||
            has_accessible_clone<ValueType>(),
        "ValueToAbstractValue(): value type must be copy constructible or "
        "have an accessible Clone() method that returns std::unique_ptr.");
    return ValueHelper(value, 1, 1);
  }

 private:
  template <typename ValueType>
  using CopyReturnType =
      decltype(ValueType(std::declval<const ValueType>()));

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
  template <typename ValueType, typename = CopyReturnType<ValueType>>
  static std::unique_ptr<AbstractValue> ValueHelper(const ValueType& value, int,
                                                    int) {
    return std::make_unique<Value<CopyReturnType<ValueType>>>(value);
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


/** Implements Drake policy for taking a concrete vector type and storing it in
a Drake numerical vector object as an AbstractValue of concrete type
`Value<BasicVector<T>>`. We call this "VectorPolicy" to distinguish it from
the "AbstractPolicy" implemented by ValueToAbstractValue.

@see ValueToAbstractValue for usage information and some restrictions.

<h4>VectorPolicy</h4>

 1. Any Eigen vector type is copied into a `Value<BasicVector<T>>`.
 2. A scalar of type T is treated as though it were an Eigen Vector1<T>.
 3. Any BasicVector or type derived from BasicVector is cloned into a
    `Value<BasicVector<T>>` although the stored object still has the
    most-derived type.
 4. Any AbstractValue object is simply cloned. The resulting clone must have
    exactly type `Value<BasicVector<T>>` or an std::logic_error is thrown.

Any other type results in an std::logic_error being thrown (at runtime). This
internal class is intended to be invoked by different user-visible APIs so
provides for the API name to be included in any generated runtime error
messages.

@tparam T The scalar type in use for vector objects. */
template <typename T>
class ValueToVectorValue {
 public:
  // Signature (1): used for any Eigen vector type, but the argument is copied
  // to a BasicVector for the returned abstract value.
  static std::unique_ptr<AbstractValue> ToAbstract(const char* api_name,
      const Eigen::Ref<const VectorX<T>>& vector) {
    unused(api_name);
    return std::make_unique<Value<BasicVector<T>>>(vector);
  }

  // Signature (2): used for a scalar of type T. The argument is copied
  // to a 1-element BasicVector for the returned abstract value.
  static std::unique_ptr<AbstractValue> ToAbstract(const char* api_name,
                                                   const T& scalar) {
    return ToAbstract(api_name, Vector1<T>(scalar));  // Use signature (1).
  }

  // Signature (3): BasicVector must be special-cased so that we use a
  // Value<BasicVector> to store a copy of the given object, even if it is from
  // a subclass of BasicVector, and even if it has its own Clone() method. The
  // actual type is preserved regardless.
  static std::unique_ptr<AbstractValue> ToAbstract(const char* api_name,
      const BasicVector<T>& vector) {
    unused(api_name);
    return std::make_unique<Value<BasicVector<T>>>(vector.Clone());
  }

  // Signature (4): used for AbstractValue or Value<U> arguments. After cloning,
  // this must be exactly type Value<BasicVector<T>>.
  static std::unique_ptr<AbstractValue> ToAbstract(const char* api_name,
      const AbstractValue& value) {
    auto cloned = value.Clone();
    if (cloned->maybe_get_value<BasicVector<T>>() != nullptr)
      return cloned;

    throw std::logic_error(
        fmt::format("{}(): the given AbstractValue containing type {} is not "
                    "suitable for storage as a Drake vector quantity.",
                    api_name, value.GetNiceTypeName()));
  }

  // The final signature exists just so we can throw an error message.

  // Signature (5): This signature won't instantiate if any of the
  // non-templatized signatures above can be used (after possible conversions).
  // If we allowed this to instantiate it would be chosen instead of performing
  // those conversions.
  template <typename ValueType,
            typename = std::enable_if_t<
                !(is_eigen_refable<ValueType>() ||
                  std::is_base_of<BasicVector<T>, ValueType>::value ||
                  std::is_base_of<AbstractValue, ValueType>::value)>>
  static std::unique_ptr<AbstractValue> ToAbstract(const char* api_name,
                                                   const ValueType& value) {
    throw std::logic_error(
        fmt::format("{}(): the given value of type {} is not "
                    "suitable for storage as a Drake vector quantity.",
                    api_name, NiceTypeName::Get<ValueType>()));
  }
};

}  // namespace internal
}  // namespace systems
}  // namespace drake

