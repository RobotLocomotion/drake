#pragma once

/* Portions copyright (c) 2015 Stanford University and the Authors.
   Authors: Michael Sherman

Licensed under the Apache License, Version 2.0 (the "License"); you may
not use this file except in compliance with the License. You may obtain a
copy of the License at http://www.apache.org/licenses/LICENSE-2.0.

(Adapted from Simbody's ClonePtr class.)
 */

#include <cstddef>
#include <iostream>
#include <memory>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/is_cloneable.h"

namespace drake {

/** @cond */

namespace copyable_unique_ptr_detail {

// This uses SFINAE to classify a particular class as "copyable". There are two
// overloads of the `is_copyable_unique_ptr_compatible_helper` struct: one is
// the default implementation and the other relies on the SFINAE and copyable
// test.
//
// The default overload reports that a class is _not_ copyable. Only if the
// class being queried passes the copyable test, will the second overload get
// created. It defines value to be true. The second overload is a more specific
// match to the helper invocation, so, if it exists, it will be instantiated by
// preference and report a true value.

template <typename V, class>
struct is_copyable_unique_ptr_compatible_helper : std::false_type {};

// This is the specific overload. The copyable condition is that it is
// "cloneable" or has a copy constructor.
template <typename V>
struct is_copyable_unique_ptr_compatible_helper<
    V, typename std::enable_if<is_cloneable<V>::value ||
        std::is_copy_constructible<V>::value>::type>
    : std::true_type {};


}  // namespace copyable_unique_ptr_detail
/** @endcond */

/**
 Test for determining if an arbitrary class is compatible with the
 copyable_unique_ptr. For a class to be compatible with the copy_unique_ptr,
 it must be copy constructible or "cloneable" (see @ref is_cloneable_doc
 "is_cloneable"). Usage:

 @code
 is_copyable_unique_ptr_compatible<TestClass>::value
 @endcode

 Evaluates to true if compatible, false otherwise. This can be used in run-time
 test, `static_assert`s, and in SFINAE voodoo.

 @see copyable_unique_ptr
 @see is_cloneable
 */
template <typename T>
using is_copyable_unique_ptr_compatible =
    copyable_unique_ptr_detail::is_copyable_unique_ptr_compatible_helper<T,
                                                                         void>;

/** A smart pointer with deep copy semantics.

 This is _similar_ to `std::unique_ptr` in that it does not permit shared
 ownership of the contained object. However, unlike `std::unique_ptr`,
 %copyable_unique_ptr supports copy and assignment operations, by insisting that
 the contained object be "copyable".
 To be copyable, the class must have either a public copy constructor, or it
 must be "cloneable" (see @ref is_cloneable_doc "is_cloneable" for definition).
 A class can be tested for compatibility using the
 @ref is_copyable_unique_ptr_compatible struct.

 Generally, the API is modeled as closely as possible on the C++ standard
 `std::unique_ptr` API and %copyable_unique_ptr<T> is interoperable with
 `unique_ptr<T>` wherever that makes sense. However, there are some differences:
   1. It always uses a default deleter.
   2. There is no array version.
   3. To allow for future copy-on-write optimizations, there is a distinction
   between writable and const access, the get() method is modified to return
   only a const pointer, with get_mutable() added to return a writable pointer.

 This class is entirely inline and has no computational or
 space overhead except when copying is required; it contains just a single
 pointer and does no reference counting.

 __Usage__

 In the simplest use case, the instantiation type will match the type of object
 it references, e.g.:
 @code
 copyable_unique_ptr<Foo> ptr = make_unique<Foo>(...);
 @endcode
 In this case, as long `Foo` is deemed compatible, the behavior will be as
 expected, i.e., when `ptr` copies, it will contain a reference to a new
 instance of `Foo`.

 %copyable_unique_ptr can also be used with polymorphic classes -- a
 %copyable_unique_ptr, instantiated on a _base_ class, references an
 instance of a _derived_ class. When copying the object, we would want the copy
 to likewise contain an instance of the derived class.  For example:

 @code
 copyable_unique_ptr<Base> cu_ptr = make_unique<Derived>();
 copyable_unique_ptr<Base> other_cu_ptr = cu_ptr;           // Triggers a copy.
 is_dynamic_castable<Derived>(cu_other_ptr.get());          // Should be true.
 @endcode

 This works for well-designed polymorphic classes.

 @warning Ill-formed polymorphic classes can lead to fatal type slicing of the
 referenced object, such that the new copy contains an instance of `Base`
 instead of `Derived`. Some mistakes that would lead to this degenerate
 behavior:
   - The `Base` class has a public copy constructor.
   - The `Base` class's Clone() implementation does not invoke the `Derived`
   class's implementation of a suitable virtual method.

 @internal For future developers:
   - the copyability of a base class does *not* imply anything about the
   copyability of a derived class. In other words, `copyable_unique_ptr<Base>`
   can be compilable while `copyable_unique_ptr<Derived>` is not.
   - Given the pointer `copyable_unique_ptr<Base> ptr(new Derived())`, even if
   this copies "correctly" (such that the copy contains an instance of
   `Derived`), this does _not_ imply that `copyable_unique_ptr<Derived>` is
   compilable.

 @see is_copyable_unique_ptr_compatible
 @tparam T   The type of the contained object, which *must* be
             @ref is_copyable_unique_ptr_compatible "compatibly copyable". May
             be an abstract or concrete type.
 */
// TODO(SeanCurtis-TRI): Consider extending this to add the Deleter as well.
template <typename T>
class copyable_unique_ptr : public std::unique_ptr<T> {
  static_assert(is_copyable_unique_ptr_compatible<T>::value,
                "copyable_unique_ptr can only be used with a 'copyable' class"
                    ", requiring either a public copy constructor or a valid "
                    "clone method of the form: `unique_ptr<T> Clone() const`.");

 public:
  /** @name                    Constructors **/
  /**@{*/

  /** Default constructor stores a `nullptr`. No heap allocation is performed.
   The empty() method will return true when called on a default-constructed
   %copyable_unique_ptr. */
  copyable_unique_ptr() noexcept : std::unique_ptr<T>() {}

  /** Given a pointer to a writable heap-allocated object, take over
   ownership of that object. No copying occurs. */
  explicit copyable_unique_ptr(T* ptr) noexcept : std::unique_ptr<T>(ptr) {}

  /** Copy constructor is deep; the new %copyable_unique_ptr object contains a
   new copy of the object in the source, created via the source object's
   copy constructor or `Clone()` method. If the source container is empty this
   one will be empty also. */
  copyable_unique_ptr(const copyable_unique_ptr& cu_ptr)
      : std::unique_ptr<T>(CopyOrNull(cu_ptr.get())) {}

  /** Copy constructor from a standard `unique_ptr` of _compatible_ type. The
   copy is deep; the new %copyable_unique_ptr object contains a new copy of the
   object in the source, created via the source object's copy constructor or
   `Clone()` method. If the source container is empty this one will be empty
   also. */
  template <typename U>
  explicit copyable_unique_ptr(const std::unique_ptr<U>& u_ptr)
      : std::unique_ptr<T>(CopyOrNull(u_ptr.get())) {}

  /** Move constructor is very fast and leaves the source empty. Ownership
   is transferred from the source to the new %copyable_unique_ptr. If the source
   was empty this one will be empty also. No heap activity occurs. */
  copyable_unique_ptr(copyable_unique_ptr&& cu_ptr) noexcept
      : std::unique_ptr<T>(cu_ptr.release()) {}

  /** Move constructor from a standard `unique_ptr`. The move is very fast and
   leaves the source empty. Ownership is transferred from the source to the new
   %copyable_unique_ptr. If the source was empty this one will be empty also. No
   heap activity occurs. */
  explicit copyable_unique_ptr(std::unique_ptr<T>&& u_ptr) noexcept
      : std::unique_ptr<T>(u_ptr.release()) {}

  /** Move construction from a compatible standard `unique_ptr`. Type `U*` must
   be implicitly convertible to type `T*`. Ownership is transferred from the
   source to the new %copyable_unique_ptr. If the source was empty this one will
   be empty also. No heap activity occurs. */
  template <typename U>
  explicit copyable_unique_ptr(std::unique_ptr<U>&& u_ptr) noexcept
      : std::unique_ptr<T>(u_ptr.release()) {}

  /**@}*/

  /** @name                   Assignment */
  /**@{*/

  /** This form of assignment replaces the currently-held object by
   the given source object and takes over ownership of the source object. The
   currently-held object (if any) is deleted. */
  copyable_unique_ptr& operator=(T* ptr) noexcept {
    std::unique_ptr<T>::reset(ptr);
    return *this;
  }

  /** This form of assignment replaces the currently-held object by a
   heap-allocated copy of the source object, created using its copy
   constructor or `Clone()` method. The currently-held object (if any) is
   deleted. */
  copyable_unique_ptr & operator=(const T& ref) {
    std::unique_ptr<T>::reset(CopyOrNull(&ref));
    return *this;
  }

  /** Copy assignment from %copyable_unique_ptr replaces the currently-held
   object by a copy of the object held in the source container, created using
   the source object's copy constructor or `Clone()` method. The currently-held
   object (if any) is deleted. If the source container is empty this one will be
   empty also after the assignment. Nothing happens if the source and
   destination are the same container. */
  copyable_unique_ptr& operator=(const copyable_unique_ptr& cu_ptr) {
    return operator=(static_cast<const std::unique_ptr<T>&>(cu_ptr));
  }

  /** Copy assignment from a compatible %copyable_unique_ptr replaces the
   currently-held object by a copy of the object held in the source container,
   created using the source object's copy constructor or `Clone()` method. The
   currently-held object (if any) is deleted. If the source container is empty
   this one will be empty also after the assignment. Nothing happens if the
   source and destination are the same container. */
  template <typename U>
  copyable_unique_ptr& operator=(const copyable_unique_ptr<U>& cu_ptr) {
    return operator=(static_cast<const std::unique_ptr<U>&>(cu_ptr));
  }

  /** Copy assignment from a standard `unique_ptr` replaces the
   currently-held object by a copy of the object held in the source container,
   created using the source object's copy constructor or `Clone()` method. The
   currently-held object (if any) is deleted. If the source container is empty
   this one will be empty also after the assignment. Nothing happens if the
   source and destination are the same container. */
  copyable_unique_ptr& operator=(const std::unique_ptr<T>& src) {
    if (&src != this) {
      // can't be same ptr unless null
      DRAKE_DEMAND((get() != src.get()) || !get());
      std::unique_ptr<T>::reset(CopyOrNull(src.get()));
    }
    return *this;
  }

  /** Copy assignment from a compatible standard `unique_ptr` replaces the
   currently-held object by a copy of the object held in the source container,
   created using the source object's copy constructor or `Clone()` method. The
   currently-held object (if any) is deleted. If the source container is empty
   this one will be empty also after the assignment. Nothing happens if the
   source and destination are the same container. */
  template <typename U>
  copyable_unique_ptr& operator=(const std::unique_ptr<U>& u_ptr) {
    // can't be same ptr unless null
    DRAKE_DEMAND((get() != u_ptr.get()) || !get());
    std::unique_ptr<T>::reset(CopyOrNull(u_ptr.get()));
    return *this;
  }

  /** Move assignment replaces the currently-held object by the source object,
   leaving the source empty. The currently-held object (if any) is deleted.
   The instance is _not_ copied. Nothing happens if the source and destination
   are the same containers. */
  copyable_unique_ptr& operator=(copyable_unique_ptr&& cu_ptr) noexcept {
    std::unique_ptr<T>::reset(cu_ptr.release());
    return *this;
  }

  /** Move assignment replaces the currently-held object by the compatible
   source object, leaving the source empty. The currently-held object (if any)
   is deleted. The instance is _not_ copied. Nothing happens if the source and
   destination are the same containers. */
  template <typename U>
  copyable_unique_ptr& operator=(copyable_unique_ptr<U>&& cu_ptr) noexcept {
    std::unique_ptr<T>::reset(cu_ptr.release());
    return *this;
  }

  /** Move assignment replaces the currently-held object by the source object,
   leaving the source empty. The currently-held object (if any) is deleted.
   The instance is _not_ copied. Nothing happens if the source and destination
   are the same containers. */
  copyable_unique_ptr& operator=(std::unique_ptr<T>&& u_ptr) noexcept {
    std::unique_ptr<T>::reset(u_ptr.release());
    return *this;
  }

  /** Move assignment replaces the currently-held object by the compatible
   source object, leaving the source empty. The currently-held object (if
   any) is deleted. The instance is _not_ copied. Nothing happens if the source
   and destination are the same containers. */
  template <typename U>
  copyable_unique_ptr& operator=(std::unique_ptr<U>&& u_ptr) noexcept {
    std::unique_ptr<T>::reset(u_ptr.release());
    return *this;
  }

  /**@}*/

  /** @name                   Observers                    */
  /**@{*/

  /** Return true if this container is empty, which is the state the container
   is in immediately after default construction and various other
   operations. */
  bool empty() const noexcept { return !(*this); }

  /** Return a const pointer to the contained object if any, or `nullptr`.
   Note that this is different than `%get()` for the standard smart pointers
   like `std::unique_ptr` which return a writable pointer. Use get_mutable()
   here for that purpose. */
  const T* get() const noexcept { return std::unique_ptr<T>::get(); }

  /** Return a writable pointer to the contained object if any, or `nullptr`.
   Note that you need write access to this container in order to get write
   access to the object it contains.

   @warning If %copyable_unique_ptr is instantiated on a const template
   parameter (e.g., `copyable_unique_ptr<const Foo>`), then get_mutable()
   returns a const pointer. */
  T* get_mutable() noexcept { return std::unique_ptr<T>::get(); }

  /**@}*/

 private:
  // Selects Clone iff there is no copy constructor and the Clone method is of
  // the expected form.
  template <typename U>
  static typename std::enable_if<
      !std::is_copy_constructible<U>::value && is_cloneable<T>::value, U*>::type
  CopyOrNullHelper(const U* ptr, int) {
    return ptr->Clone().release();
  }

  // Default to copy constructor if present.
  template <typename U>
  static
  U* CopyOrNullHelper(const U* ptr, ...) {
    return new U(*ptr);
  }

  // If src is non-null, clone it; otherwise return nullptr.
  static T* CopyOrNull(const T *ptr) {
    return ptr ? CopyOrNullHelper(ptr, 1) : nullptr;
  }
};

/** Output the system-dependent representation of the pointer contained
 in a copyable_unique_ptr object. This is equivalent to `os << p.get();`.
 @relates copyable_unique_ptr */
template <class charT, class traits, class T>
inline std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os,
    const copyable_unique_ptr<T>& cu_ptr) {
  os << cu_ptr.get();
  return os;
}

}  // namespace drake
