#pragma once

#include <type_traits>

#include "drake/common/drake_compat.h"

namespace drake {
namespace systems {
namespace transmogrifier {

/// A templated traits class for whether S<T> can transmogrify into S<U>; the
/// default value is true for all values of S, T, and U.  Particular classes
/// may specialize this template to indicate whether the framework should
/// support transmogrification for certain combinations of S, T, and U.
///
/// In supported cases, the "transmogrification copy constructor" for those
/// types will be used; in unsupported cases, the "transmogrification copy
/// constructor" will not even be mentioned, so that S need not even compile
/// for certain values of T or U.  (See TransmogrifierTag for details on the
/// "transmogrification copy constructor".)
///
/// @tparam S is the type to transmogrify
template <template <typename> class S>
struct Traits {
  /// @tparam U is the donor scalar type (to transmogrify from)
  /// @tparam T is the resulting scalar type (to transmogrify into)
  template <typename T, typename U>
  using supported = std::true_type;
};

/// A concrete traits class providing sugar to disable support for
/// symbolic::Expression.  For example, if MySystem does not support the
/// symbolic expression scalar type, it should specialize Traits as follows:
///
/// @code
/// namespace drake {
/// namespace systems {
/// namespace transmogrifier {
/// template <> struct Traits<MySystem> : public NonSymbolicTraits {};
/// }  // namespace transmogrifier
/// }  // namespace systems
/// }  // namespace drake
/// @endcode
struct NonSymbolicTraits {
  template <typename T, typename U>
  using supported = typename std::conditional<
    !std::is_same<T, symbolic::Expression>::value &&
    !std::is_same<U, symbolic::Expression>::value,
    std::true_type, std::false_type>::type;
};

/// XXX Document me.  Only allow double as the donor type.
struct FromDoubleTraits {
  template <typename T, typename U>
  using supported = typename std::conditional<
    std::is_same<U, double>::value,
    std::true_type, std::false_type>::type;
};

}  // namespace transmogrifier
}  // namespace systems
}  // namespace drake
