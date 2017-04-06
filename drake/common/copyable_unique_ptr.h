#pragma once

/* Portions copyright (c) 2015 Stanford University and the Authors.
   Authors: Michael Sherman

Licensed under the Apache License, Version 2.0 (the "License"); you may
not use this file except in compliance with the License. You may obtain a
copy of the License at http://www.apache.org/licenses/LICENSE-2.0. */

#include <iostream>
#include <cstddef>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_cloneable.h"

namespace drake {

/** Smart pointer with deep copy semantics.

 This is similar to `std::unique_ptr` in that it does not permit shared
 ownership of the contained object. However, unlike `std::unique_ptr`,
 %clone_unique_ptr supports copy and assignment operations, by insisting that
 the contained object have a Clone() method that returns a pointer to a
 heap-allocated deep copy of the *concrete* object. The API is modeled as
 closely as possible on the C++11 `std::unique_ptr` API. However, it always uses
 a default deleter.

 There is one notable exception, to mark a careful distinction between writable
 and const access, and for compatibility with cow_unique_ptr, the get() method
 is modified to return only a const pointer, with get_mutable() added to return
 a writable pointer. This class is entirely inline and has no computational or
 space overhead except when cloning is required; it contains just a single
 pointer and does no reference counting.

 @tparam T   The type of the contained object, which *must* have a `Clone()`
             method. May be an abstract or concrete type.
 @see clone_unqiue_ptr
 */
template <typename T>
class copyable_unique_ptr {
 public:
  typedef T  element_type; ///< Type of the contained object.
  typedef T* pointer;      ///< Type of a pointer to the contained object.
  typedef T& reference;    ///< Type of a reference to the contained object.

  static_assert(std::is_copy_constructible<T>::value ||
                is_cloneable<T>::value,
                "copyable_unique_ptr can only be used with a 'copyable' class"
                ", requiring either a public copy constructor or a valid "
                "clone method of the form: `unique_ptr<T> Clone() const`.");

  /** @name                    Constructors **/
  /**@{**/

  /** Default constructor stores a `nullptr`. No heap allocation is performed.
   The empty() method will return true when called on a default-constructed
   %copyable_unique_ptr.
   **/
  copyable_unique_ptr() noexcept : p(nullptr) {}

  /** Constructor from `nullptr` is the same as the default constructor.
   This is an implicit conversion that allows `nullptr` to be used to
   initialize a %copyable_unique_ptr.
   **/
  copyable_unique_ptr(std::nullptr_t) noexcept : copyable_unique_ptr() {}

  /** Given a pointer to a writable heap-allocated object, take over
   ownership of that object. The `Clone()` method is *not* invoked.
   **/
  explicit copyable_unique_ptr(T* x) noexcept : p(x) {}

  /** Given a pointer to a read-only object, create a new heap-allocated copy
   of that object via its `Clone()` method and make this %copyable_unique_ptr
   the owner of the copy. Ownership of the original object is not affected. If
   the supplied pointer is null, the resulting %copyable_unique_ptr will be
   empty.
   **/
  explicit copyable_unique_ptr(const T* x)
      : copyable_unique_ptr(cloneOrNull(x)) {}

  /** Given a read-only reference to an object, create a new heap-allocated
   copy of that object via its `Clone()` method and make this
   %copyable_unique_ptr object the owner of the copy. Ownership of the original
   object is not affected.
   **/
  explicit copyable_unique_ptr(const T& x) : copyable_unique_ptr(&x) {}

  /** Copy constructor is deep; the new %copyable_unique_ptr object contains a
   new copy of the object in the source, created via the source object's
   `Clone()` method. If the source container is empty this one will be empty
   also.
   **/
  copyable_unique_ptr(const copyable_unique_ptr& src) : p(cloneOrNull(src.p)) {}

  /** Deep copy construction from a compatible %copyable_unique_ptr. Type `U*`
   must be implicitly convertible to type `T*`. The new %copyable_unique_ptr
   object contains a new copy of the object in the source, created via the
   source object's `Clone()` method. If the source container is empty this one
   will be empty also.
   **/
  template <class U>
  copyable_unique_ptr(const copyable_unique_ptr<U>& src)
      : p(cloneOrNull(src.p)) {}

  /** Move constructor is very fast and leaves the source empty. Ownership
   is transferred from the source to the new %copyable_unique_ptr. If the source
   was empty this one will be empty also. No heap activity occurs.
   **/
  copyable_unique_ptr(copyable_unique_ptr&& src) noexcept : p(src.release()) {}

  /** Move construction from a compatible %copyable_unique_ptr. Type `U*` must
   be implicitly convertible to type `T*`. Ownership is transferred from the
   source to the new %copyable_unique_ptr. If the source was empty this one will
   be empty also. No heap activity occurs.
   **/
  template <class U>
  copyable_unique_ptr(copyable_unique_ptr<U>&& src) noexcept : p(src.release()) {}
  /**@}**/

  /** @name                   Assignment **/
  /**@{**/

  /** Copy assignment replaces the currently-held object by a copy of the object
   held in the source container, created using the source object's `Clone()`
   method. The currently-held object (if any) is deleted. If the source
   container is empty this one will be empty also after the assignment. Nothing
   happens if the source and destination are the same container.
   **/
  copyable_unique_ptr& operator=(const copyable_unique_ptr& src) {
    if (&src != this) {
      DRAKE_ASSERT((p != src.p) || !p);  // can't be same ptr unless null
      reset(cloneOrNull(src.p));
    }
    return *this;
  }

  /** Copy assignment from a compatible %copyable_unique_ptr. Type `U*` must be
   implicitly convertible to type `T*`. The currently-held object is replaced
   by a copy of the object held in the source container, created using the
   source object's `Clone()` method. The currently-held object (if any) is
   deleted. If the source container is empty this one will be empty also after
   the assignment.
   **/
  template <class U>
  copyable_unique_ptr& operator=(const copyable_unique_ptr<U>& src) {
    // The source can't be the same container as this one since they are
    // different types. The managed pointers should never be the same either
    // since copyable_unique_ptrs represent unique ownership. (OK if both
    // nullptr.)
    DRAKE_ASSERT((p != static_cast<const T*>(src.p)) || !p);

    reset(cloneOrNull(src.p));
    return *this;
  }

  /** Move assignment replaces the currently-held object by the source object,
   leaving the source empty. The currently-held object (if any) is deleted.
   The `Clone()` method is *not* invoked. Nothing happens if the source and
   destination are the same containers.
   **/
  copyable_unique_ptr& operator=(copyable_unique_ptr&& src) noexcept {
    if (&src != this) {
      DRAKE_ASSERT((p != src.p) || !p); // can't be same ptr unless null
      reset(src.p); src.p = nullptr;
    }
    return *this;
  }

  /** Move assignment from a compatible %copyable_unique_ptr replaces the
   currently-held object by the source object, leaving the source empty. Type U*
   must be implicitly convertible to type T*. The currently-held object (if any)
   is deleted. The `Clone()` method is *not* invoked.
   **/
  template <class U>
  copyable_unique_ptr& operator=(copyable_unique_ptr<U>&& src) noexcept {
    // The source can't be the same container as this one since they are
    // different types. The managed pointers should never be the same either
    // since copyable_unique_ptrs represent unique ownership. (OK if both
    // nullptr.)
    DRAKE_ASSERT((p != static_cast<const T*>(src.p)) || !p);
    reset(src.p); src.p = nullptr;
    return *this;
  }

  /** This form of assignment replaces the currently-held object by a
   heap-allocated copy of the source object, created using its `Clone()`
   method. The currently-held object (if any) is deleted.
   **/
  copyable_unique_ptr& operator=(const T& x)
  {   reset(cloneOrNull(&x)); return *this; }

  /** This form of assignment replaces the currently-held object by
   the given source object and takes over ownership of the source object. The
   currently-held object (if any) is deleted.
   **/
  copyable_unique_ptr& operator=(T* x) noexcept
  {   reset(x); return *this; }
  /**@}**/

  /** @name                    Destructor **/
  /**@{**/
  /** Destructor deletes the contained object.
  @see reset() **/
  ~copyable_unique_ptr() noexcept {reset();}
  /**@}**/

  /** @name                     Accessors **/
  /**@{**/

  /** Return a const pointer to the contained object if any, or `nullptr`.
   Note that this is different than `%get()` for the standard smart pointers
   like `std::unique_ptr` which return a writable pointer. Use get_mutable()
   here for that purpose.
   @see get_mutable(), get_ref()
   **/
  const T* get() const noexcept {return p;}

  /** Return a writable pointer to the contained object if any, or `nullptr`.
   Note that you need write access to this container in order to get write
   access to the object it contains.
   @see get(), get_mutable_ref()
   **/
  T* get_mutable() noexcept {return p;}

  /** Return a const reference to the contained object. Don't call this if
   this container is empty.
   @see get() **/
  const T& get_ref() const {
    DRAKE_ASSERT(!empty());
    return *get();
  }

  /** Return a writable reference to the contained object. Don't call this if
   this container is empty.
   @see get_mutable()
   **/
  T& get_mutable_ref() {
    DRAKE_ASSERT(!empty());
    return *get_mutable();
  }

  /** Dereference a const pointer to the contained object. This will fail if
   the container is empty. **/
  const T* operator->() const { return &get_ref(); }

  /** Dereference a writable pointer to the contained object. This will fail
   if the container is empty. **/
  T* operator->() { return &get_mutable_ref(); }

  /** This "dereference" operator returns a const reference to the contained
   object. This will fail if the container is empty. **/
  const T& operator*() const {return get_ref();}

  /** Return a writable reference to the contained object. This will fail if
   the container is empty. **/
  T& operator*() {return get_mutable_ref();}
  /**@}**/

  /** @name                      Utility Methods **/
  /**@{**/

  /** Make this container empty if it isn't already, destructing the contained
   object if there is one. The container is restored to its default-constructed
   state.
   @see empty()
   **/
  void reset() noexcept {
    delete p;
    p = nullptr;
  }

  /** Replace the contents of this container with the supplied heap-allocated
   object, taking over ownership of that object and deleting the current one
   first if necessary. Nothing happens if the supplied pointer is the same
   as the one already being managed.
   **/
  void reset(T* x) noexcept {
    if (x != p) {
      delete p;
      p = x;
    }
  }

  /** Swap the contents of this %copyable_unique_ptr with another one, with
   ownership changing hands but no copying performed. This is very fast;
   no heap activity occurs. Both containers must have been instantiated with
   the identical type.
   **/
  void swap(copyable_unique_ptr& other) noexcept {
    std::swap(p, other.p);
  }

  /** Return true if this container is empty, which is the state the container
   is in immediately after default construction and various other
   operations.
   **/
  bool empty() const noexcept {return !p;}

  /** This is a conversion to type bool that returns true if the container is
   non-null (that is, not empty). **/
  explicit operator bool() const noexcept {return !empty();}

  /** Remove the contained object from management by this container and
   transfer ownership to the caller. A writable pointer to the object is
   returned. No object destruction occurs. This %copyable_unique_ptr is left
   empty.
   **/
  T* release() noexcept {
    T* save = p;
    p = nullptr;
    return save;
  }
  /**@}**/

 private:
  template <class U> friend class copyable_unique_ptr;

  template <typename U>
  static
  typename std::enable_if<!std::is_copy_constructible<U>::value &&
                          is_cloneable<T>::value, U*>::type
  cloneOrNull_(const U* src, int) {
    return src->Clone().release();
  };

  template <typename U>
  static
  U* cloneOrNull_(const U* src, ...) {
    return new U(*src);
  }

  // If src is non-null, clone it; otherwise return nullptr.
  static T* cloneOrNull(const T* src) {
    return src ? cloneOrNull_(src, 1) : nullptr;
  }

  T*      p;          // this may be null
};

//==============================================================================
//                       drake namespace-scope functions
//==============================================================================
// These namespace-scope functions will be resolved by the compiler using
// "Koenig lookup" which examines the arguments' namespaces first.
// See Herb Sutter's discussion here:
// http://www.gotw.ca/publications/mill08.htm.

/** This is an overload of the STL std::swap() algorithm which uses the
 cheap built-in swap() member of the copyable_unique_ptr class. (This function
 is defined in the `drake` namespace.)
 @relates copyable_unique_ptr
 **/
template <class T> inline void
swap(copyable_unique_ptr<T>& p1, copyable_unique_ptr<T>& p2) noexcept {
  p1.swap(p2);
}

/** Output the system-dependent representation of the pointer contained
 in a copyable_unique_ptr object. This is equivalent to `os << p.get();`.
 @relates copyable_unique_ptr
 **/
template <class charT, class traits, class T>
inline std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, const copyable_unique_ptr<T>& p) {
  os << p.get();
  return os;
}

/** Compare for equality the managed pointers contained in two compatible
 copyable_unique_ptr containers. Returns `true` if the pointers refer to
 the same object or if both are null. It must be possible for one of the
 pointer types `T*` and `U*` to be implicitly converted to the other.
 @relates copyable_unique_ptr
 **/
template <class T, class U>
inline bool operator==(const copyable_unique_ptr<T>& lhs,
                       const copyable_unique_ptr<U>& rhs) {
  return lhs.get() == rhs.get();
}

/** Comparison against `nullptr`; same as `lhs.empty()`. 
 @relates copyable_unique_ptr
 **/
template <class T>
inline bool operator==(const copyable_unique_ptr<T>& lhs, std::nullptr_t) {
  return lhs.empty();
}

/** Comparison against `nullptr`; same as `rhs.empty()`.
 @relates copyable_unique_ptr
 **/
template <class T>
inline bool operator==(std::nullptr_t, const copyable_unique_ptr<T>& rhs) {
  return rhs.empty();
}

/** Less-than operator for two compatible copyable_unique_ptr containers,
 comparing the *pointers*, not the *objects* they point to. Returns `true` if
 the lhs pointer tests less than the rhs pointer. A null pointer tests less than
 any non-null pointer. It must be possible for one of the pointer types `T*` and
 `U*` to be implicitly converted to the other.
 @relates copyable_unique_ptr
 **/
template <class T, class U>
inline bool operator<(const copyable_unique_ptr<T>& lhs,
                      const copyable_unique_ptr<U>& rhs) {
  return lhs.get() < rhs.get();
}

/** Less-than comparison against a `nullptr`. A null pointer tests less than any
 non-null pointer and equal to another null pointer, so this method always
 returns `false`.
 @relates copyable_unique_ptr
 **/
template <class T>
inline bool operator<(const copyable_unique_ptr<T>& lhs, std::nullptr_t) {
  return false;
}

/** Less-than comparison of a `nullptr` against this container. A null
 pointer tests less than any non-null pointer and equal to another null pointer,
 so this method returns `true` unless the container is empty.
 @relates copyable_unique_ptr
 **/
template <class T>
inline bool operator<(std::nullptr_t, const copyable_unique_ptr<T>& rhs) {
  return !rhs.empty();
}

// These functions are derived from operator== and operator<.

/** Pointer inequality test defined as `!(lhs==rhs)`.
 @relates copyable_unique_ptr
 **/
template <class T, class U>
inline bool operator!=(const copyable_unique_ptr<T>& lhs,
                       const copyable_unique_ptr<U>& rhs) {
  return !(lhs == rhs);
}

/** `nullptr` inequality test defined as `!(lhs==nullptr)`.
 @relates copyable_unique_ptr
 **/
template <class T>
inline bool operator!=(const copyable_unique_ptr<T>& lhs, std::nullptr_t) {
  return !(lhs == nullptr);
}

/** `nullptr` inequality test defined as `!(nullptr==rhs)`.
 @relates copyable_unique_ptr
 **/
template <class T>
inline bool operator!=(std::nullptr_t, const copyable_unique_ptr<T>& rhs) {
  return !(nullptr == rhs);
}

/** Pointer greater-than test defined as `rhs < lhs`.
 @relates copyable_unique_ptr
 **/
template <class T, class U>
inline bool operator>(const copyable_unique_ptr<T>& lhs,
                      const copyable_unique_ptr<U>& rhs) {
  return rhs < lhs;
}

/** `nullptr` greater-than test defined as `nullptr < lhs`.
 @relates copyable_unique_ptr
 **/
template <class T>
inline bool operator>(const copyable_unique_ptr<T>& lhs, std::nullptr_t) {
  return nullptr < lhs;
}

/** `nullptr` greater-than test defined as `rhs < nullptr`.
 @relates copyable_unique_ptr
 **/
template <class T>
inline bool operator>(std::nullptr_t, const copyable_unique_ptr<T>& rhs) {
  return rhs < nullptr;
}

/** Pointer greater-or-equal test defined as `!(lhs < rhs)`.
 @relates copyable_unique_ptr
 **/
template <class T, class U>
inline bool operator>=(const copyable_unique_ptr<T>& lhs,
                       const copyable_unique_ptr<U>& rhs) {
  return !(lhs < rhs);
}

/** `nullptr` greater-or-equal test defined as `!(lhs < nullptr)`.
 @relates copyable_unique_ptr
 **/
template <class T>
inline bool operator>=(const copyable_unique_ptr<T>& lhs, std::nullptr_t) {
  return !(lhs < nullptr);
}

/** `nullptr` greater-or-equal test defined as `!(nullptr < rhs)`.
 @relates copyable_unique_ptr
 **/
template <class T>
inline bool operator>=(std::nullptr_t, const copyable_unique_ptr<T>& rhs) {
  return !(nullptr < rhs);
}

/** Pointer less-or-equal test defined as `!(rhs < lhs)` (note reversed
 arguments).
 @relates copyable_unique_ptr
 **/
template <class T, class U>
inline bool operator<=(const copyable_unique_ptr<T>& lhs,
                       const copyable_unique_ptr<U>& rhs) {
  return !(rhs < lhs);
}

/** `nullptr` less-or-equal test defined as `!(nullptr < lhs)` (note reversed
 arguments).
 @relates copyable_unique_ptr
 **/
template <class T>
inline bool operator<=(const copyable_unique_ptr<T>& lhs, std::nullptr_t) {
  return !(nullptr < lhs);
}

/** `nullptr` less-or-equal test defined as `!(rhs < nullptr)` (note reversed
 arguments).
 @relates copyable_unique_ptr
 **/
template <class T>
inline bool operator<=(std::nullptr_t, const copyable_unique_ptr<T>& rhs) {
  return !(rhs < nullptr);
}

}  // namespace drake
