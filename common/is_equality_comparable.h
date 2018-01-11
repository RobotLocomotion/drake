#pragma once

#include <type_traits>

#include "drake/common/unused.h"

#ifndef DRAKE_DOXYGEN_CXX

namespace drake {

namespace is_equality_comparable_detail {

// Default case; assumes that a class is *not* equality comparable.
template <typename T, typename = void>
struct is_equality_comparable_helper : std::false_type { };

// Special sauce for SFINAE. Only compiles if it can finds the method
// `operator==`. If this exists, the is_equality_comparable implicitly
// prefers this overload over the default overload.
template <typename T>
struct is_equality_comparable_helper<T, typename std::enable_if<true,
    decltype(unused(std::declval<T&>() == std::declval<T&>()),
    (void)0)>::type> : std::true_type {};

}  // namespace is_equality_comparable_detail

/** @endcond */

/**
 @anchor is_equality_comparable_doc
 Provides method for determining at run time if a class is comparable using
 the equality operator (==).

 __Usage__

 This gets used like `type_traits` functions (e.g., `is_copy_constructible`,
 `is_same`, etc.) To determine if a class is equality comparable simply invoke:

 @code
 bool value = drake::is_equality_comparable<Foo>::value;
 @endcode

 If `Foo` is equality comparable, it will evalute to true. It can also be used
 in compile-time tests (e.g., SFINAE and `static_assert`s):

 @code
 static_assert(is_equality_comparable<Foo>::value, "This method requires its "
               "classes to be equality comparable.");
 @endcode

 __Definition of "equality comparability"__

 To be equality comparable, the class `Foo` must have a public method of the
 form:

 @code
 bool Foo::operator==(const Foo&) const;
 @endcode

 or a definition external to the class of the form:

 @code
 bool Foo::operator==(const Foo&, const Foo&);
 @endcode

 @tparam  T  The class to test for equality comparability.
 */
template <typename T>
using is_equality_comparable =
    is_equality_comparable_detail::is_equality_comparable_helper<T, void>;

}  // namespace drake

#endif
