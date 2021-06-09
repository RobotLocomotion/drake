#pragma once

#include <type_traits>

#include "drake/common/symbolic.h"

namespace drake {
namespace systems {
namespace scalar_conversion {

/// A templated traits class for whether an `S<U>` can be converted into an
/// `S<T>`; the default value is true for all values of `S`, `T`, and `U`.
/// Particular scalar-dependent classes (`S`) may specialize this template to
/// indicate whether the framework should support conversion for any given
/// combination of `T` and `U`.
///
/// When `Traits<S>::supported<T, U>` is `std::true_type`, the
/// "scalar-converting copy constructor" that relates `S`, `T`, and `U` will be
/// used.  That constructor takes the form of, e.g., when `S` is `Foo`:
///
/// @code
/// template <typename T>
/// class Foo {
///   template <typename U>
///   explicit Foo(const Foo<U>& other);
/// };
/// @endcode
///
/// See @ref system_scalar_conversion for detailed background and examples
/// related to scalar-type conversion support.
///
/// When `Traits<S>::supported<T, U>` is `std::false_type`, the
/// `S<T>::S(const S<U>&)` scalar-conversion copy constructor will not even be
/// mentioned by the framework, so that `S` need not even compile for certain
/// values of `T` and `U`.
///
/// @tparam S is the scalar-templated type to copy
template <template <typename> class S>
struct Traits {
  /// @tparam T is the resulting scalar type (to convert into)
  /// @tparam U is the donor scalar type (to convert from)
  template <typename T, typename U>
  using supported = std::true_type;
};

/// A concrete traits class providing sugar to disable support for symbolic
/// evaluation (i.e., the symbolic::Expression scalar type).
///
/// For example, if MySystem does not support the symbolic expression scalar
/// type, it could specialize Traits as follows:
///
/// @code
/// namespace drake {
/// namespace systems {
/// namespace scalar_conversion {
/// template <> struct Traits<MySystem> : public NonSymbolicTraits {};
/// }  // namespace scalar_conversion
/// }  // namespace systems
/// }  // namespace drake
/// @endcode
struct NonSymbolicTraits {
  template <typename T, typename U>
  using supported = typename std::bool_constant<
    !std::is_same_v<T, symbolic::Expression> &&
    !std::is_same_v<U, symbolic::Expression>>;
};

/// A concrete traits class providing sugar to support for converting only from
/// the `double` scalar type.  For example, if a MySystem<symbolic::Expression>
/// cannot be converted into a MySystem<double>, it could specialize Traits as
/// follows:
///
/// @code
/// namespace drake {
/// namespace systems {
/// namespace scalar_conversion {
/// template <> struct Traits<MySystem> : public FromDoubleTraits {};
/// }  // namespace scalar_conversion
/// }  // namespace systems
/// }  // namespace drake
/// @endcode
struct FromDoubleTraits {
  template <typename T, typename U>
  using supported = typename std::bool_constant<std::is_same_v<U, double>>;
};

/// Converts a scalar `U u` to its corresponding scalar `T t`.  When U == T,
/// the scalar is unchanged.  When demoting Expression to non-Expression,
/// throws when there are unbound variables.  In all other cases, information
/// beyond the double value (e.g., possible derivatives) might be discarded.
template <typename T, typename U>
struct ValueConverter {
  T operator()(const U& u) const {
    return ExtractDoubleOrThrow(u);
  }
};
template <typename T>
struct ValueConverter<T, T> {
  using U = T;
  T operator()(const U& u) const {
    return u;
  }
};

}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake
