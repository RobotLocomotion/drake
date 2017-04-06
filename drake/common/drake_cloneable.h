#pragma once

#include <memory>
#include <type_traits>

namespace drake {
/*! @file
 Provides method for determining at run time if a class is "cloneable". To be
 cloneable, it must have a public method of the form:

 `unique_ptr<Foo> Clone() const;`

 This can be used in static assertions as in:

 static_assert(is_cloneable<Foo>::value, "This method requires its classes to "
                                         "be cloneable.");
 */

// Helper template for SFINAE.
template <class...>
using void_t = void;

// Default case; assumes that a class is *not* cloneable.
template <typename T, class>
struct is_cloneable_ : std::false_type {};

// Special sauce for SFINAE. Only compiles if it can finds the method
// `unique_ptr<T> T::Clone()`. It can be const or non-const.
template <typename T>
struct is_cloneable_<T, typename std::enable_if<std::is_same<
                            decltype(std::declval<const T>().Clone()),
                            std::unique_ptr<T>>::value>::type>
    : std::true_type {};

// The "public" interface.  Should be invoked as:
//   is_cloneable<Foo>::value
// which evaluates to true or false, as appropriate.
template <typename T>
using is_cloneable = is_cloneable_<T, void_t<>>;

}  // namespace drake
