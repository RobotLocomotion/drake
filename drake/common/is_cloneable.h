#pragma once

#include <memory>
#include <type_traits>


namespace drake {

/** @cond */

namespace is_cloneable_detail {

// Default case; assumes that a class is *not* cloneable.
template <typename T, class>
struct is_cloneable_helper : std::false_type {};

// Special sauce for SFINAE. Only compiles if it can finds the method
// `unique_ptr<U> T::Clone() const`, where U* can be assigned to T* (meaning
// T and U are the same or U is derived from T).
template <typename T>
struct is_cloneable_helper<
    T,
    typename std::enable_if<std::is_convertible<
        decltype(std::declval<const T>().Clone().release())&, T*>::value>::type>
    : std::true_type {};

}  // namespace is_cloneable_detail

/** @endcond */

/**
 @anchor is_cloneable_doc
 Provides method for determining at run time if a class is "cloneable".

 __Usage__

 This gets used like `type_traits` functions (e.g., `is_copy_constructible`,
 `is_same`, etc.) To determine if a class is cloneable simply invoke:

 @code
 bool value = drake::is_cloneable<Foo>::value;
 @endcode

 If `Foo` is cloneable, it will evalute to true. It can also be used in
 compile-time tests (e.g., SFINAE and `static_assert`s):

 @code
 static_assert(is_cloneable<Foo>::value, "This method requires its classes to "
                                         "be cloneable.");
 @endcode

 __Definition of "cloneability"__

 To be cloneable, the class `Foo` must have a public method of the form:

 @code
 unique_ptr<Bar> Foo::Clone() const;
 @endcode

 where `Foo` is derived from `Bar` or `Bar` _is_ `Foo`. The pointer contained in
 the `unique_ptr` must point to a heap-allocated deep copy of the _concrete_
 object. This test can confirm the proper signature, but cannot confirm the
 heap-allocated deep copy. A Clone() method that doesn't return such a copy of
 the _concrete_ object, should be considered a malformed function.

 @tparam  T  The class to test for cloneability.
 */
template <typename T>
using is_cloneable =
    is_cloneable_detail::is_cloneable_helper<T, void>;

}  // namespace drake
