#pragma once

#include <memory>
#include <type_traits>

/*! @file Provides method for determining at run time if a class is "cloneable".
 */
namespace drake {

/*! @cond */

// Helper template for SFINAE.
template <class...>
using void_t = void;

// Default case; assumes that a class is *not* cloneable.
template <typename T, class>
struct is_cloneable_ : std::false_type {};

// Special sauce for SFINAE. Only compiles if it can finds the method
// `unique_ptr<U> T::Clone() const`. Where T is derived from U (where T is
// derived from itself, by convention).
template <typename T>
struct is_cloneable_<
    T,
    typename std::enable_if<std::is_assignable<
        decltype(std::declval<const T>().Clone().release())&, T*>::value>::type>
    : std::true_type {};

/*! @endcond */

/*!
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

 where `Foo` is derived from `Bar` (and, by convention, `Foo` is derived from
 itself). The pointer contained in the `unique_ptr` must point to a
 heap-allocated deep copy of the _concrete_ object. This test can confirm the
 proper signature, but cannot confirm the heap-allocated deep copy. A Clone()
 method that doesn't return such a copy of the _concrete_ object, should be
 considered a malformed function.

 To put it another way, is_cloneable<Foo>::value is true if the following code
 compiles:

 @code
 const auto ptr = new Foo(...);
 auto clone_ptr = ptr->Clone();
 clone_ptr = ptr;
 // However, this next line may or may not compile. Cloneability does not depend
 // on this line compiling.  In this case, `ptr` is of type `Foo*`, `clone_ptr`
 // is of type `Bar*` and `Foo` ≠ `Bar`.
 ptr = clone_ptr;
 @endcode

 @tparam  T  The class to test for cloneability.
 */
template <typename T>
using is_cloneable = is_cloneable_<T, void_t<>>;

}  // namespace drake
