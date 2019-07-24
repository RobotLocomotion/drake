#pragma once

#include <memory>
#include <type_traits>

namespace drake {

/** @cond */

namespace is_cloneable_internal {

// Default case; assumes that a class is *not* cloneable.
template <typename T, class>
struct is_cloneable_helper : std::false_type {};

// Special sauce for SFINAE. Only compiles if it can finds the method
// `unique_ptr<T> T::Clone() const`. If this exists, the is_cloneable implicitly
// prefers this overload over the default overload.
template <typename T>
struct is_cloneable_helper<
    T,
    typename std::enable_if<std::is_same<
        decltype(std::declval<const T>().Clone().release()),
        typename std::remove_const<T>::type*>::value>::type>
    : std::true_type {};

}  // namespace is_cloneable_internal

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

 If `Foo` is cloneable, it will evaluate to true. It can also be used in
 compile-time tests (e.g., SFINAE and `static_assert`s):

 @code
 static_assert(is_cloneable<Foo>::value, "This method requires its classes to "
                                         "be cloneable.");
 @endcode

 __Definition of "cloneability"__

 To be cloneable, the class `Foo` must have a _public_ method of the form:
 @code
 unique_ptr<Foo> Foo::Clone() const;
 @endcode
 Note that "friend" access for the %is_cloneable-using class is not sufficient.
 The `Foo::Clone()` method must actually be public.

 <!-- Developer note: if you change or extend the definition of an acceptable
      clone method here, be sure to consider whether
      copyable_unique_ptr::can_clone() should be changed as well. -->

 The pointer contained in the returned `unique_ptr` must point to a
 heap-allocated deep copy of the _concrete_ object. This test can confirm the
 proper signature, but cannot confirm the heap-allocated deep copy. A Clone()
 method that doesn't return such a copy of the _concrete_ object should be
 considered a malformed function.

 @warning It is important to note, that a `Clone()` method that returns a
 `unique_ptr` to a _super_ class is _not_ sufficient to be cloneable. In other
 words the presence of:
 @code
 unique_ptr<Base> Derived::Clone() const;
 @endcode
 will not make the `Derived` class cloneable.

 @tparam  T  The class to test for cloneability.
 */
template <typename T>
using is_cloneable =
    is_cloneable_internal::is_cloneable_helper<T, void>;

}  // namespace drake
