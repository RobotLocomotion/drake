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

namespace drake {

// TODO(SeanCurtis-TRI): Consider extending this to add the Deleter as well.
/** A smart pointer with deep copy semantics.

 This is _similar_ to `std::unique_ptr` in that it does not permit shared
 ownership of the contained object. However, unlike `std::unique_ptr`,
 %copyable_unique_ptr supports copy and assignment operations, by insisting that
 the contained object be "copyable". To be copyable, the class must have either
 an accessible copy constructor, or it must have an accessible clone method
 with signature @code
   std::unique_ptr<Foo> Clone() const;
 @endcode
 where Foo is the type of the managed object. By "accessible" we mean either
 that the copy constructor or clone method is public, or
 `friend copyable_unique_ptr<Foo>;` appears in Foo's class declaration.

 <!-- Developer note: if you change or extend the definition of an acceptable
      clone method here, be sure to consider whether drake::is_cloneable should
      be changed as well. -->

 Generally, the API is modeled as closely as possible on the C++ standard
 `std::unique_ptr` API and %copyable_unique_ptr<T> is interoperable with
 `unique_ptr<T>` wherever that makes sense. However, there are some differences:

 1. It always uses a default deleter.
 2. There is no array version.
 3. To allow for future copy-on-write optimizations, there is a distinction
    between writable and const access, the get() method is modified to return
    only a const pointer, with get_mutable() added to return a writable pointer.
    Furthermore, derefencing (operator*()) a mutable pointer will give a mutable
    reference (in so far as T is not declared const), and dereferencing a
    const pointer will give a const reference.

 This class is entirely inline and has no computational or space overhead except
 when copying is required; it contains just a single pointer and does no
 reference counting.

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
 is_dynamic_castable<Derived>(other_cu_ptr.get());          // Should be true.
 @endcode

 This works for well-designed polymorphic classes.

 @warning Ill-formed polymorphic classes can lead to fatal type slicing of the
 referenced object, such that the new copy contains an instance of `Base`
 instead of `Derived`. Some mistakes that would lead to this degenerate
 behavior:

   - The `Base` class's Clone() implementation does not invoke the `Derived`
   class's implementation of a suitable virtual method.

 <!--
 For future developers:
   - the copyability of a base class does *not* imply anything about the
   copyability of a derived class. In other words, `copyable_unique_ptr<Base>`
   can be compilable while `copyable_unique_ptr<Derived>` is not.
   - Given the pointer `copyable_unique_ptr<Base> ptr(new Derived())`, even if
   this copies "correctly" (such that the copy contains an instance of
   `Derived`), this does _not_ imply that `copyable_unique_ptr<Derived>` is
   compilable.
 -->

 @tparam T   The type of the contained object, which *must* be copyable as
             defined above. May be an abstract or concrete type.
 */
template <typename T>
class copyable_unique_ptr : public std::unique_ptr<T> {
 public:
  /** @name                    Constructors */
  /**@{*/

  /** Default constructor stores a `nullptr`. No heap allocation is performed.
   The empty() method will return true when called on a default-constructed
   %copyable_unique_ptr. */
  copyable_unique_ptr() noexcept : std::unique_ptr<T>() {}

  /** Given a raw pointer to a writable heap-allocated object, take over
   ownership of that object. No copying occurs. */
  explicit copyable_unique_ptr(T* raw) noexcept : std::unique_ptr<T>(raw) {}

  /** Constructs a unique instance of T as a copy of the provided model value.
   */
  explicit copyable_unique_ptr(const T& value)
      : std::unique_ptr<T>(CopyOrNull(&value)) {}

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
  copyable_unique_ptr& operator=(T* raw) noexcept {
    std::unique_ptr<T>::reset(raw);
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

  // TODO(SeanCurtis-TRI): Consider adding some debug assertions about whether
  // T is const or not. If so, it would be nice to give feedback that calling
  // the mutable version makes no sense.
  /** Return a writable pointer to the contained object if any, or `nullptr`.
   Note that you need write access to this container in order to get write
   access to the object it contains.

   @warning If %copyable_unique_ptr is instantiated on a const template
   parameter (e.g., `copyable_unique_ptr<const Foo>`), then get_mutable()
   returns a const pointer. */
  T* get_mutable() noexcept { return std::unique_ptr<T>::get(); }

  // TODO(15344) We need to shore up this const correctness hole. Rather than an
  //  is-a relationship, we need some alternative relationship that will provide
  //  the same functionality but not be upcastable. One possibility is to own
  //  an unique_ptr and forward various APIs. Another is to implement from
  //  scratch. The current "is-A" relationship was intended so that the
  //  copyable_unique_ptr could be used where unique_ptrs are used. What would
  //  the impact of such a change in the relationship be to Drake and Drake
  //  users?

  /** Return a const reference to the contained object. Note that this is
   different from `std::unique_ptr::operator*()` which would return a non-const
   reference (if `T` is non-const), even if the container itself is const. For
   a const %copyable_unique_ptr will always return a const reference to its
   contained value.

   @warning Currently %copyable_unique_ptr is a std::unique_ptr. As such, a
   const copyable_unique_ptr<Foo> can be upcast to a const unique_ptr<Foo> and
   the parent's behavior will provide a mutable reference. This is strongly
   discouraged and will break as the implementation of this class changes to
   shore up this gap in the const correctness protection.

   @pre `this != nullptr` reports `true`. */
  const T& operator*() const { return *get(); }

  /** Return a writable reference to the contained object (if T is itself not
   const). Note that you need write access to this container in order to get
   write access to the object it contains.

   We *strongly* recommend, that, if dereferencing a %copyable_unique_ptr
   without the intention of mutating the underlying value, prefer to dereference
   a *const* %copyable_unique_ptr (or use *my_ptr.get()) and not a mutable
   %copyable_unique_ptr. As "copy-on-write" behavior is introduced in the
   future, this recommended practice will prevent unwanted copies of the
   underlying value.

   If %copyable_unique_ptr is instantiated on a const template parameter (e.g.,
   `copyable_unique_ptr<const Foo>`), then operator*() must return a const
   reference.

   @pre `this != nullptr` reports `true`. */
  T& operator*() { return *get_mutable(); }

  /**@}*/
 private:
  // The can_copy() and can_clone() methods must be defined within the
  // copyable_unique_ptr class so that they have the same method access as
  // the class does. That way we can use them to determine whether
  // copyable_unique_ptr can get access. That precludes using helper classes
  // like drake::is_cloneable because those may have different access due to an
  // explicit friend declaration giving copyable_unique_ptr<Foo> access to Foo's
  // private business. The static_assert below ensures that at least one of
  // these must return true.

  // SFINAE magic explanation. We're combining several tricks here:
  // (1) "..." as a parameter type is a last choice; an exact type match is
  //     preferred in overload resolution.
  // (2) Use a dummy template parameter U that is always just T but defers
  //     instantiation so that substitution failure is not fatal.
  // (3) We construct a non-evaluated copy constructor and Clone method in
  //     templatized methods to prevent instantiation if the needed method
  //     doesn't exist or isn't accessible. If instantiation is successful,
  //     we produce an exact-match method that trumps the "..."-using method.
  // (4) Make these constexpr so they can be used in static_assert.

  // True iff type T provides a copy constructor that is accessible from
  // %copyable_unique_ptr<T>. Invoke with `can_copy(1)`; the argument is used
  // to select the right method.
  static constexpr bool can_copy(...) { return false; }

  // If this instantiates successfully it will be the preferred method called
  // when an integer argument is provided.
  template <typename U = T>
  static constexpr std::enable_if_t<
      std::is_same_v<decltype(U(std::declval<const U&>())), U>,
      bool>
  can_copy(int) {
    return true;
  }

  // True iff type T provides a `Clone()` method with the appropriate
  // signature (see class documentation) that is accessible from
  // %copyable_unique_ptr<T>. Invoke with `can_clone(1)`; the argument is used
  // to select the right method.
  static constexpr bool can_clone(...) { return false; }

  // If this instantiates successfully it will be the preferred method called
  // when an integer argument is provide.
  template <typename U = T>
  static constexpr std::enable_if_t<
      std::is_same_v<decltype(std::declval<const U>().Clone()),
                     std::unique_ptr<std::remove_const_t<U>>>,
      bool>
  can_clone(int) {
    return true;
  }

  // If src is non-null, clone it; otherwise return nullptr.
  // If both can_copy() and can_clone() return true, we will prefer the Clone()
  // function over the copy constructor.
  // The caller has ownership over the return value.
  static T* CopyOrNull(const T* raw) {
    constexpr bool check_can_clone = can_clone(1);
    constexpr bool check_can_copy = can_copy(1);
    static_assert(
        check_can_clone || check_can_copy,
        "copyable_unique_ptr<T> can only be used with a 'copyable' class T, "
        "requiring either a copy constructor or a Clone method of the form "
        "'unique_ptr<T> Clone() const'.");
    if (raw == nullptr) {
      return nullptr;
    }
    if constexpr (check_can_clone) {
      return raw->Clone().release();
    } else {
      return new T(*raw);
    }
  }
};

/** Output the system-dependent representation of the pointer contained
 in a copyable_unique_ptr object. This is equivalent to `os << p.get();`.
 @relates copyable_unique_ptr */
template <class charT, class traits, class T>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os,
    const copyable_unique_ptr<T>& cu_ptr) {
  os << cu_ptr.get();
  return os;
}

}  // namespace drake
