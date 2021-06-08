#pragma once

#include <type_traits>

#include "drake/common/unused.h"

namespace drake {

#ifndef DRAKE_DOXYGEN_CXX

namespace is_less_than_comparable_internal {

// Default case; assumes that a class is *not* less-than comparable.
template <typename T, typename = void>
struct is_less_than_comparable_helper : std::false_type { };

// Special sauce for SFINAE. Only compiles if it can finds the method
// `operator<`. If this exists, the is_less_than_comparable implicitly
// prefers this overload over the default overload.
template <typename T>
struct is_less_than_comparable_helper<T, typename std::enable_if_t<true,
    decltype(unused(std::declval<T&>() < std::declval<T&>()),
    (void)0)>> : std::true_type {};

}  // namespace is_less_than_comparable_internal

/** @endcond */

/**
 @anchor is_less_than_comparable_doc
 Provides method for determining at run time if a class is comparable using
 the less-than operator (<).

 __Usage__

 This gets used like `type_traits` functions (e.g., `is_copy_constructible`,
 `is_same`, etc.) To determine if a class is less-than comparable simply invoke:

 @code
 bool value = drake::is_less_than_comparable<Foo>::value;
 @endcode

 If `Foo` is less-than comparable, it will evaluate to true. It can also be used
 in compile-time tests (e.g., SFINAE and `static_assert`s):

 @code
 static_assert(is_less_than_comparable<Foo>::value, "This method requires its "
               "classes to be less-than comparable.");
 @endcode

 __Definition of "less-than comparability"__

 To be less-than comparable, the class `Foo` must have a public method of the
 form:

 @code
 bool Foo::operator<(const Foo&) const;
 @endcode

 or a definition external to the class of the form:

 @code
 bool operator<(const Foo&, const Foo&);
 @endcode

 @tparam  T  The class to test for less-than comparability.
 */
template <typename T>
using is_less_than_comparable =
    is_less_than_comparable_internal::is_less_than_comparable_helper<T, void>;

}  // namespace drake

#endif
