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

namespace copyable_unique_ptr_detail {

/** @cond */
template <typename V, class>
struct is_copyable_unique_ptr_compatible_helper : std::false_type {};

template <typename V>
struct is_copyable_unique_ptr_compatible_helper<
    V, typename std::enable_if<is_cloneable<V>::value ||
        std::is_copy_constructible<V>::value>::type>
    : std::true_type {};

/** @endcond */

}  // namespace copyable_unique_ptr_detail

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

/** Smart pointer with deep copy semantics.

 This is _similar_ to `std::unique_ptr` in that it does not permit shared
 ownership of the contained object. However, unlike `std::unique_ptr`,
 %copyable_unique_ptr supports copy and assignment operations, by insisting that
 the contained object be "copyable".
 To be copyable, the class must have either a public copy constructor, or it
 must be "cloneable" (see @ref is_cloneable_doc "is_cloneable" for definition).
 A class can be tested for compatibility using the
 @ref is_copyable_unique_ptr_compatible struct.

 Generally, the API is modeled as closely as possible on the C++11
 `std::unique_ptr` API. However, there are some exceptions.
   1. It always uses a default deleter.
   2. There is no array version.
   3. To allow for future copy-on-write optimizations, there is a distinction
   between writable and const access, the get() method is modified to return
   only a const pointer, with get_mutable() added to return a writable pointer.
   There are also corresponding get_ref() and get_mutable_ref() with the
   analogous semantics.

 This class is entirely inline and has no computational or
 space overhead except when copying is required; it contains just a single
 pointer and does no reference counting.

 __Usage without polymorphism__

 In many typical use cases, the pointer specialization type will match the type
 of object it references, e.g.:
 @code
 copyable_unique_ptr<Foo> ptr = make_unique<Foo>(...);
 @endcode
 In this case, as long `Foo` is deemed compatible, the behavior will be as
 expected.

 __Usage _with_ polymorphism__

 There will be cases, where the pointer specialization type will be different
 from the object referenced -- the referenced object would be a concrete derived
 type, e.g.:
 @code
 copyable_unique_ptr<Base> ptr = make_unique<Derived>(...);
 @endcode
 The requirement that Base be copyable puts constraints on the definitions of
 `Base` and `Foo` in order for this to behave as expected.

   1. If the `Base` class has a _public_ copy constructor, that copy constructor
      will be invoked. In the best case, if `Derived` provides no additional
      members, it will be upcast into an instance of `Base` in the copy.
      At worst, if `Derived` _does_ add members, it will be type sliced into
      an instance of `Base`.
   2. As noted in @ref is_cloneable_doc "is_cloneable", both the `Base` and
      `Derived` classes must have Clone methods of the form:
      `unique_ptr<T> T::Clone() const`.

 We recommend the following to resolve these constraints if you need to use
 %copyable_unique_ptr in a polymorphic context:

   1. Make the `Base` such that it has no public copy constructor.
   2. Implement the `Clone()` method using NVI (e.g., `Clone()` and a protected
      `virtual DoClone()`).
   3. In the `Derived` class, hide the `Base` class's declaration of `Clone()`
      with one that returns the appropriate type.
   4. The `Derived` class *can* have a public copy constructor.

  These principles are illustrated in this code:

  @code
  struct Base {
    virtual ~Base() {}
    // Clone method satisifies the is_cloneable test.
    std::unique_ptr<Base> Clone() const {
        return std::unique_ptr<Base>(DoClone());
    }
   protected:
    // Protected copy constructor will not be used by copyable_unique_ptr.
    Base(const Base& b) = default;
    // Virtual DoClone() provides the desired polymorphism.
    virtual Base* DoClone() const {
        return new Base(*this);
    }
  };

  struct Derived {
    // Public copy constructor acceptable -- just don't sub-class this.
    Derived(const Derived& d) = default;
    // This declaration hides Base::Clone() and satisfies is_cloneable.
    std::unique_ptr<Derived> Clone() const {
        return std::unique_ptr<Derived>(DoClone());
    }
   protected:
    // Override DoClone() to create a real copy of this class.
    Base* DoClone() const override {
        return new Derived(*this);
    }
  };
  @endcode

 Using this type of relationship, one can specialize %copyable_unique_ptr on
 `Base` and `Derived` classes, and reference a `Derived` instance in a
 `%copyable_unique_ptr<Base>` and still create correct copies.

 @see is_copyable_unique_ptr_compatible
 @tparam T   The type of the contained object, which *must* be "copyable". May
             be an abstract or concrete type.
 */
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
  copyable_unique_ptr() : std::unique_ptr<T>() {}

  /** Given a pointer to a writable heap-allocated object, take over
   ownership of that object. The `Clone()` method is *not* invoked. */
  explicit copyable_unique_ptr(T* ptr) : std::unique_ptr<T>(ptr) {}

  /** Copy constructor is deep; the new %copyable_unique_ptr object contains a
   new copy of the object in the source, created via the source object's
   copy constructor or `Clone()` method. If the source container is empty this
   one will be empty also. */
  copyable_unique_ptr(const copyable_unique_ptr& src)
      : std::unique_ptr<T>(CopyOrNull(src.get())) {}

  /** Copy constructor from a standard `unique_ptr` of _compatible_ type. The
   copy is deep; the new %copyable_unique_ptr object contains a new copy of the
   object in the source, created via the source object's copy constructor or
   `Clone()` method. If the source container is empty this one will be empty
   also. */
  template <typename U>
  explicit copyable_unique_ptr(const std::unique_ptr<U>& src)
      : std::unique_ptr<T>(CopyOrNull(src.get())) {}

  /** Move constructor is very fast and leaves the source empty. Ownership
   is transferred from the source to the new %copyable_unique_ptr. If the source
   was empty this one will be empty also. No heap activity occurs. */
  copyable_unique_ptr(copyable_unique_ptr&& ptr)
      : std::unique_ptr<T>(ptr.release()) {}

  /** Move constructor from a standard `unique_ptr`. The move is very fast and
   leaves the source empty. Ownership is transferred from the source to the new
   %copyable_unique_ptr. If the source was empty this one will be empty also. No
   heap activity occurs. */
  explicit copyable_unique_ptr(std::unique_ptr<T>&& p)
      : std::unique_ptr<T>(p.release()) {}

  /** Move construction from a compatible standard `unique_ptr`. Type `U*` must
   be implicitly convertible to type `T*`. Ownership is transferred from the
   source to the new %copyable_unique_ptr. If the source was empty this one will
   be empty also. No heap activity occurs. */
  template <typename U>
  explicit copyable_unique_ptr(std::unique_ptr<U>&& ptr)
      : std::unique_ptr<T>(ptr.release()) {}

  /**@}**/

  /** @name                   Assignment **/
  /**@{**/

  /** This form of assignment replaces the currently-held object by
   the given source object and takes over ownership of the source object. The
   currently-held object (if any) is deleted. */
  copyable_unique_ptr& operator=(T* x) noexcept {
    std::unique_ptr<T>::reset(x);
    return *this;
  }

  /** This form of assignment replaces the currently-held object by a
   heap-allocated copy of the source object, created using its copy
   constructor or `Clone()` method. The currently-held object (if any) is
   deleted. */
  copyable_unique_ptr & operator=(const T& x) {
    std::unique_ptr<T>::reset(CopyOrNull(&x));
    return *this;
  }

  /** Copy assignment from %copyable_unique_ptr replaces the currently-held
   object by a copy of the object held in the source container, created using
   the source object's copy constructor or `Clone()` method. The currently-held
   object (if any) is deleted. If the source container is empty this one will be
   empty also after the assignment. Nothing happens if the source and
   destination are the same container. */
  copyable_unique_ptr& operator=(const copyable_unique_ptr& src) {
    if (&src != this) {
      // can't be same ptr unless null
      DRAKE_ASSERT((get() != src.get()) || !get());
      std::unique_ptr<T>::reset(CopyOrNull(src.get()));
    }
    return *this;
  }

  /** Copy assignment from a compatible %copyable_unique_ptr replaces the
   currently-held object by a copy of the object held in the source container,
   created using the source object's copy constructor or `Clone()` method. The
   currently-held object (if any) is deleted. If the source container is empty
   this one will be empty also after the assignment. Nothing happens if the
   source and destination are the same container. */
  template <typename U>
  copyable_unique_ptr& operator=(const copyable_unique_ptr<U>& src) {
    // can't be same ptr unless null
    DRAKE_ASSERT((get() != src.get()) || !get());
    std::unique_ptr<T>::reset(CopyOrNull(src.get()));

    return *this;
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
      DRAKE_ASSERT((get() != src.get()) || !get());
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
  copyable_unique_ptr& operator=(const std::unique_ptr<U>& src) {
    // can't be same ptr unless null
    DRAKE_ASSERT((get() != src.get()) || !get());
    std::unique_ptr<T>::reset(CopyOrNull(src.get()));
    return *this;
  }

  /** Move assignment replaces the currently-held object by the source object,
   leaving the source empty. The currently-held object (if any) is deleted.
   The instance is _not_ copied. Nothing happens if the source and destination
   are the same containers. */
  copyable_unique_ptr& operator=(copyable_unique_ptr&& p) {
    std::unique_ptr<T>::reset(p.release());
    return *this;
  }

  /** Move assignment replaces the compatible, currently-held object by the
   source object, leaving the source empty. The currently-held object (if any)
   is deleted. The instance is _not_ copied. Nothing happens if the source and
   destination are the same containers. */
  template <typename U>
  copyable_unique_ptr& operator=(copyable_unique_ptr<U>&& p) {
    std::unique_ptr<T>::reset(p.release());
    return *this;
  }

  /** Move assignment replaces the currently-held object by the source object,
   leaving the source empty. The currently-held object (if any) is deleted.
   The instance is _not_ copied. Nothing happens if the source and destination
   are the same containers. */
  copyable_unique_ptr& operator=(std::unique_ptr<T>&& p) {
    std::unique_ptr<T>::reset(p.release());
    return *this;
  }

  /** Move assignment of compatible type replaces the currently-held object by
   the source object, leaving the source empty. The currently-held object (if
   any) is deleted. The instance is _not_ copied. Nothing happens if the source
   and destination are the same containers. */
  template <typename U>
  copyable_unique_ptr& operator=(std::unique_ptr<U>&& p) {
    std::unique_ptr<T>::reset(p.release());
    return *this;
  }

  /**@}**/

  /** @name                   Observers                    **/
  /**@{**/

  /** Return true if this container is empty, which is the state the container
   is in immediately after default construction and various other
   operations. */
  bool empty() const noexcept { return !(*this); }

  /** Return a const pointer to the contained object if any, or `nullptr`.
   Note that this is different than `%get()` for the standard smart pointers
   like `std::unique_ptr` which return a writable pointer. Use get_mutable()
   here for that purpose.
   @see get_mutable(), get_ref() */
  const T* get() const noexcept { return std::unique_ptr<T>::get(); }

  /** Return a writable pointer to the contained object if any, or `nullptr`.
   Note that you need write access to this container in order to get write
   access to the object it contains.
   @see get(), get_mutable_ref() */
  T* get_mutable() noexcept { return std::unique_ptr<T>::get(); }

  /** Return a const reference to the contained object. Throws an exception if
   the container is empty.
   @see get() **/
  const T& get_ref() const {
    if (empty())
      throw std::logic_error(
          "Trying to access a reference for a null copyable_unique_ptr.");
    return *get();
  }

  /** Return a writable reference to the contained object. Throws an exception
   if the container is empty.
   @see get_mutable() */
  T& get_mutable_ref() {
    if (empty())
      throw std::logic_error(
          "Trying to access a reference for a null copyable_unique_ptr.");
    return *get_mutable();
  }

  /**@}**/

 private:
  // Selects Clone iff there is no copy constructor and the Clone method is of
  // the expected form.
  template <typename U>
  static typename std::enable_if<
      !std::is_copy_constructible<U>::value && is_cloneable<T>::value, U*>::type
  CopyOrNullHelper(const U* src, int) {
    return src->Clone().release();
  }

  // Default to copy constructor if present.
  template <typename U>
  static
  U* CopyOrNullHelper(const U* src, ...) {
    return new U(*src);
  }

  // If src is non-null, clone it; otherwise return nullptr.
  static T* CopyOrNull(const T *src) {
    return src ? CopyOrNullHelper(src, 1) : nullptr;
  }
};

/** Output the system-dependent representation of the pointer contained
 in a copyable_unique_ptr object. This is equivalent to `os << p.get();`.
 @relates copyable_unique_ptr */
template <class charT, class traits, class T>
inline std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, const copyable_unique_ptr<T>& p) {
  os << p.get();
  return os;
}

}  // namespace drake
