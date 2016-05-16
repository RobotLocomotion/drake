#pragma once

#include <cassert>
#include <cstddef>
#include <functional>
#include <iosfwd>
#include <memory>
#include <type_traits>
#include <utility>

#include "drake/drakeCommon_export.h"

namespace drake {

/* Adapted from Simbody's ClonePtr class, under these terms:
Portions copyright (c) 2005-15 Stanford University.
Licensed under the Apache License, Version 2.0 (the "License"); you may
not use this file except in compliance with the License. You may obtain a
copy of the License at http://www.apache.org/licenses/LICENSE-2.0. */

/** A `unique_ptr` that retains a non-owned reference pointer to the original
object after ownership is transferred elsewhere.

@tparam T   The type of the contained object, which must not be an array type.

This smart pointer handles ownership identically to `std::unique_ptr` -- it does
not permit shared ownership of the managed object. The API here is modeled as
closely as possible on the C++17 `std::unique_ptr` API except:
- it always uses a default deleter, and
- there is no specialization for array types.

Other differences are due to the additional functionality. Unlike
`std::unique_ptr`, `%drake::unique_ptr_ref` continues to provide access to
the previously-managed object after ownership has been transferred. Unlike
`std::shared_ptr`, that retained reference does not extend the managed object's
lifetime, which is always limited by the lifetime of its unique owner.

This is particularly useful to provide a clean API for construction of a complex
object composed of many elements (for example, a tree of articulated bodies and
joints). The caller creates an owned, concrete element (a body or joint, say)
using `%drake::unique_ptr_ref` and then transfers ownership to the larger
object, but retains a reference to the concrete object. That pointer can be used
later for accessing the concrete element, without a lookup or downcast. Usage
example: @code{.cc}
  // Assume:
  class PinJoint    : public Joint { public: double GetAngle(); };
  class SliderJoint : public Joint { public: double GetDisplacement(); };
  class Tree {
   public:  void AddJoint(std::unique_ptr<Joint> joint);
   private: std::vector<std::unique_ptr<Joint>> joints_;
  };
  Tree tree;

  // Create some concrete Joint elements.
  auto pin    = drake::make_unique_ref<PinJoint>(pin_axis);
  auto slider = drake::make_unique_ref<SlidingJoint>(slider_axis);

  // (At this point pin and slider own the managed objects; early termination
  // or failure to add these to a Tree would clean up with no leaks.)

  // Transfer ownership to tree; pin and slider will be deleted when tree is.
  tree.AddJoint(pin.move());
  tree.AddJoint(slider.move());

  // Can still reference concrete class methods (as long as tree exists).
  pin.GetAngle();
  slider.GetDisplacement();
@endcode

Details
-------
Unlike `std::unique_ptr`, this class supports copy construction and
assignment, but only a non-owned reference is copied, so there is still a
unique owner of the managed object after copy or assign. Move construction and
assignment transfers ownership but the source retains an unowned reference.
A `%unique_ptr_ref` can also be moved to an `std::unique_ptr` in which case
the destination has ownership and the source retains a reference.

This class is entirely inline and has no computational overhead. Space overhead
is one boolean to remember whether we have ownership of the managed object.
Because of alignment requirements, the actual space used by `%unique_ptr_ref`
will typically be the size of two pointers.

Specializations are provided for method `drake::swap<unique_ptr_ref>()` and
class `std::hash<drake::unique_ptr_ref>` with behavior defined similarly to
the corresponding specializations for `std::unique_ptr`.

@see make_unique_ref() **/
template <class T>
class unique_ptr_ref {
  /** @cond **/  // static_assert confuses doxygen.
  static_assert(!std::is_array<T>::value,
                "drake::unique_ptr_ref<T> does not support array types T.");
  /** @endcond **/

 public:
  typedef T element_type;  ///< Type of the contained object.
  typedef T* pointer;      ///< Type of a pointer to the contained object.
  typedef T& reference;    ///< Type of a reference to the contained object.

  /** @name                    Constructors **/
  /**@{**/

  /** Default constructor stores a `nullptr`.
  No heap allocation is performed. The `empty()` method will return `true` when
  called on a default-constructed %unique_ptr_ref. The `is_owner()` method will
  return `false`. **/
  constexpr unique_ptr_ref() noexcept {}

  /** Constructor from `nullptr` is the same as the default constructor.
  This is an implicit conversion that allows `nullptr` to be used to initialize
  a %unique_ptr_ref. **/
  constexpr unique_ptr_ref(std::nullptr_t) noexcept : unique_ptr_ref() {}

  /** Given a pointer to a writable heap-allocated object, take over
  ownership of that object. Prefer `make_unique_ref()` rather than
  invoking the constructor directly; see the example in the class documentation.
  @see make_unique_ref() **/
  explicit unique_ptr_ref(T* p) noexcept : p_{p}, is_owner_{true} {}

  /** Copy constructor copies the pointer but the new %unique_ptr_ref
  object does not have ownership, regardless of whether the source object did.
  If the source container is empty this one will be empty also. **/
  unique_ptr_ref(const unique_ptr_ref& source) noexcept : p_{source.p_} {}

  /** Copy construction from a compatible %unique_ptr_ref.
  Type `U*` must be implicitly convertible to type `T*`. Ownership is not
  transferred, but the new `%unique_ptr_ref` will have a pointer to the object
  that is managed by @p source. **/
  template <typename U>
  unique_ptr_ref(const unique_ptr_ref<U>& source) noexcept : p_{source.p_} {}

  /** Move constructor copies the pointer from the source, and transfers
  ownership if the source owned the object. The source will not have ownership
  after this constructor is executed, but will retain its copy of the pointer.
  If the source was empty this one will be empty also. **/
  unique_ptr_ref(unique_ptr_ref&& source) noexcept
      : p_{source.p_}, is_owner_{source.is_owner_} {
    source.is_owner_ = false;
  }

  /** Move construction from a compatible `%unique_ptr_ref` rvalue reference.
  Type `U*` must be implicitly convertible to type `T*`. Ownership is
  transferred from the source to the new `%unique_ptr_ref`. If the source was
  empty this one will be empty also. **/
  template <class U>
  unique_ptr_ref(unique_ptr_ref<U>&& source) noexcept
      : p_{source.p_}, is_owner_{source.is_owner_} {
    source.is_owner_ = false;
  }

  /** Move construction from a compatible `std::unique_ptr` rvalue reference.
  Type `U*` must be implicitly convertible to type `T*`. Ownership is
  transferred from the source source to the new `%unique_ptr_ref`. If the source
  was empty this will be empty also. **/
  template <class U>
  unique_ptr_ref(
      std::unique_ptr<U>&& source) noexcept  // NOLINT(runtime/explicit)
      : p_{source.release()},
        is_owner_{p_ != nullptr} {}
  /**@}**/

  /** @name                   Assignment **/
  /**@{**/

  /** Copy assignment copies the pointer held in the source container, but does
  not take ownership. The currently-held object (if any) is deleted if owned. If
  the source container is empty this one will be empty also after assignment.
  Nothing happens if the source and destination manage the same pointer. **/
  unique_ptr_ref& operator=(const unique_ptr_ref& source) noexcept {
    if (source.p_ != p_) {
      reset();  // now null, unowned
      p_ = source.p_;
    }
    return *this;
  }

  /** Copy assignment from a compatible %unique_ptr_ref. Type `U*` must be
  implicitly convertible to type `T*`. The currently-held object (if any) is
  deleted if owned. If the source container is empty this one will be empty also
  after assignment. Nothing happens if the source and destination manage the
  same pointer. **/
  template <class U>
  unique_ptr_ref& operator=(const unique_ptr_ref<U>& source) noexcept {
    if (source.p_ != p_) {
      reset();  // now null, unowned
      p_ = source.p_;
    }
    return *this;
  }

  /** Move assignment replaces the currently-held object by the source object,
  and transfers ownership if the source owned the object. The currently-held
  object (if any) is deleted. If source and this manage the same pointer, only
  the ownership changes. **/
  unique_ptr_ref& operator=(unique_ptr_ref&& source) noexcept {
    if (source.p_ != p_) {
      reset();  // now null, unowned
      p_ = source.p_;
    }
    is_owner_ = source.is_owner_;
    source.is_owner_ = false;
    return *this;
  }

  /** Move assignment from a compatible %unique_ptr_ref rvalue reference. Type
  `U*` must be implicitly convertible to type `T*`. Replaces the currently-held
  object by the source object, and transfers ownership if the source owned the
  object. The currently-held object (if any) is deleted. The source is left with
  the same pointer, but loses ownership if it had it. If source and `this`
  manage the same pointer, only the ownership changes. **/
  template <class U>
  unique_ptr_ref& operator=(unique_ptr_ref<U>&& src) noexcept {
    if (source.p_ != p_) {
      reset();  // now null, unowned
      p_ = source.p_;
    }
    is_owner_ = source.is_owner_;
    source.is_owner_ = false;
    return *this;
  }

  /** Assignment to a `nullptr` restores this container to its
  default-constructed state. **/
  unique_ptr_ref& operator=(nullptr_t) noexcept {
    reset();
    return *this;
  }
  /**@}**/

  /** @name                    Destructor **/
  /**@{**/
  /** Destructor deletes the contained object if it is owned.
  @see reset() **/
  ~unique_ptr_ref() noexcept { reset(); }
  /**@}**/

  /** @name                     Accessors **/
  /**@{**/

  /** Return a pointer to the contained object if any, or `nullptr`. Note that
  the pointed-to object may be owned by this container or not; use `is_owner()`
  if you need to know. **/
  T* get() const noexcept { return p_; }

  /** Dereference the pointer to the contained object. Behavior is undefined if
  the container is empty. **/
  T* operator->() const noexcept { return p_; }

  /** This "dereference" operator returns a reference to the contained
  object. Behavior is undefined if the container is empty. May throw an
  exception if `T` has a throwing `operator*`. **/
  T& operator*() const { return *p_; }
  /**@}**/

  /** @name                      Utility Methods **/
  /**@{**/

  /** Transfer ownership to a temporary (rvalue) `std::unique_ptr` which can be
  moved into a retained `std::unique_ptr` via move construction or assignment.
  If this container does not have ownership, the returned `std::unique_ptr` will
  be empty. **/
  std::unique_ptr<T> move() noexcept { return std::unique_ptr<T>(release()); }

  /** Make this container empty if it isn't already, deleting the contained
  object if there is one and it is owned. The container is restored to its
  default-constructed state.
  @see empty(), is_owner() **/
  void reset() noexcept {
    if (is_owner_) {
      delete p_;
      is_owner_ = false;
    }
    p_ = nullptr;
  }

  /** Replace the contents of this container with a compatible heap-allocated
  object, taking over ownership of that object and deleting the current one
  first if necessary. Type `U*` must be implicitly convertible to `T*`. If the
  supplied pointer is the same as the one already being managed, then no delete
  occurs but this container takes over ownership if it didn't have ownership
  before. If the supplied pointer is null this container is restored to its
  default-constucted state. **/
  template <class U>
  void reset(U* p) noexcept {
    if (p_ != p) {
      reset();
      p_ = p;
    }
    is_owner_ = (p_ != nullptr);
  }

  /** Swap the contents of this %unique_ptr_ref with another one, with ownership
  changing hands. Both containers must have been instantiated with the identical
  type. **/
  void swap(unique_ptr_ref& other) noexcept {
    std::swap(p_, other.p_);
    std::swap(is_owner_, other.is_owner_);
  }

  /** Return true if the contained pointer is null. **/
  bool empty() const noexcept { return !p_; }

  /** Return true if this container has ownership of the object it is
  referencing. An empty container will return `false`. **/
  bool is_owner() const noexcept { return is_owner_; }

  /** This is a conversion to type bool that returns true if the container is
  not empty. **/
  explicit operator bool() const noexcept { return !empty(); }

  /** If this container owns the object to which it refers, this method returns
  the pointer and transfers ownership to the caller. Otherwise it returns
  `nullptr`, even if it contains a non-null (but unowned) pointer. No object
  destruction occurs. The pointer is retained for future reference, but
  `is_owner()` will return `false` after this method is called. **/
  T* release() noexcept {
    if (is_owner_) {
      is_owner_ = false;
      return p_;
    }
    return nullptr;
  }
  /**@}**/

 private:
  template <typename U>
  friend class unique_ptr_ref;

  T* p_{nullptr};
  bool is_owner_{false};
};

//==============================================================================
//                       drake namespace-scope functions
//==============================================================================
// These namespace-scope functions will be resolved by the compiler using
// "Koenig lookup" which examines the arguments' namespaces first.
// See Herb Sutter's discussion here:
// http://www.gotw.ca/publications/mill08.htm.

/** Convenience method for creating a `drake::unique_ptr_ref` without
repetition of the managed type. Typical use: @code{.cc}
  auto myObj = drake::make_unique_ref<Object>(a1,a2);

  drake::unique_ptr_ref<Object> myObj(new Object(a1,a2)); // equivalent
@endcode
where `a1,a2` represent any number of arguments to be passed to one of Object's 
constructors. In general, this method is equivalent to @code{.cc}
  drake::unique_ptr_ref<T> myObj(new T(std::forward<Args>(args)...))
@endcode
@see std::make_unique()
@relates unique_ptr_ref **/
template <class T, class... Args>
unique_ptr_ref<T> make_unique_ref(Args&&... args) {
  return unique_ptr_ref<T>(new T(std::forward<Args>(args)...));
}

/** This is an overload of the STL std::swap() algorithm which uses the
cheap built-in swap() member of the unique_ptr_ref class. (This method
is defined in the `drake` namespace.)
@relates unique_ptr_ref **/
template <class T>
inline void swap(unique_ptr_ref<T>& p1, unique_ptr_ref<T>& p2) noexcept {
  p1.swap(p2);
}

/** Output the system-dependent representation of the pointer contained
in a unique_ptr_ref object. This is equivalent to `os << p.get();`.
@relates unique_ptr_ref **/
template <class charT, class traits, class T>
inline std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, const unique_ptr_ref<T>& p) {
  os << p.get();
  return os;
}

/** Compare for equality the managed pointers contained in two compatible
unique_ptr_ref containers. Returns `true` if the pointers refer to
the same object or if both are null. It must be possible for one of the
pointer types `T*` and `U*` to be implicitly converted to the other.
@relates unique_ptr_ref **/
template <class T, class U>
inline bool operator==(const unique_ptr_ref<T>& lhs,
                       const unique_ptr_ref<U>& rhs) {
  return lhs.get() == rhs.get();
}

/** Comparison against `nullptr`; same as `lhs.empty()`.
@relates unique_ptr_ref **/
template <class T>
inline bool operator==(const unique_ptr_ref<T>& lhs, std::nullptr_t) {
  return lhs.empty();
}

/** Comparison against `nullptr`; same as `rhs.empty()`.
@relates unique_ptr_ref **/
template <class T>
inline bool operator==(std::nullptr_t, const unique_ptr_ref<T>& rhs) {
  return rhs.empty();
}

/** Less-than operator for two compatible unique_ptr_ref containers,
comparing the *pointers*, not the *objects* they point to. Returns `true` if the
lhs pointer tests less than the rhs pointer. A null pointer tests less than any
non-null pointer. It must be possible for one of the pointer types `T*` and
`U*` to be implicitly converted to the other.
@relates unique_ptr_ref **/
template <class T, class U>
inline bool operator<(const unique_ptr_ref<T>& lhs,
                      const unique_ptr_ref<U>& rhs) {
  return lhs.get() < rhs.get();
}

/** Less-than comparison against a `nullptr`. A null pointer tests less than any
non-null pointer and equal to another null pointer, so this method always
returns `false`.
@relates unique_ptr_ref **/
template <class T>
inline bool operator<(const unique_ptr_ref<T>& lhs, std::nullptr_t) {
  return false;
}

/** Less-than comparison of a `nullptr` against this container. A null
pointer tests less than any non-null pointer and equal to another null pointer,
so this method returns `true` unless the container is empty.
@relates unique_ptr_ref **/
template <class T>
inline bool operator<(std::nullptr_t, const unique_ptr_ref<T>& rhs) {
  return !rhs.empty();
}

// These functions are derived from operator== and operator<.

/** Pointer inequality test defined as `!(lhs==rhs)`.
@relates unique_ptr_ref **/
template <class T, class U>
inline bool operator!=(const unique_ptr_ref<T>& lhs,
                       const unique_ptr_ref<U>& rhs) {
  return !(lhs == rhs);
}
/** `nullptr` inequality test defined as `!(lhs==nullptr)`.
@relates unique_ptr_ref **/
template <class T>
inline bool operator!=(const unique_ptr_ref<T>& lhs, std::nullptr_t) {
  return !(lhs == nullptr);
}
/** `nullptr` inequality test defined as `!(nullptr==rhs)`.
@relates unique_ptr_ref **/
template <class T>
inline bool operator!=(std::nullptr_t, const unique_ptr_ref<T>& rhs) {
  return !(nullptr == rhs);
}

/** Pointer greater-than test defined as `rhs < lhs`.
@relates unique_ptr_ref **/
template <class T, class U>
inline bool operator>(const unique_ptr_ref<T>& lhs,
                      const unique_ptr_ref<U>& rhs) {
  return rhs < lhs;
}
/** `nullptr` greater-than test defined as `nullptr < lhs`.
@relates unique_ptr_ref **/
template <class T>
inline bool operator>(const unique_ptr_ref<T>& lhs, std::nullptr_t) {
  return nullptr < lhs;
}

/** `nullptr` greater-than test defined as `rhs < nullptr`.
@relates unique_ptr_ref **/
template <class T>
inline bool operator>(std::nullptr_t, const unique_ptr_ref<T>& rhs) {
  return rhs < nullptr;
}

/** Pointer greater-or-equal test defined as `!(lhs < rhs)`.
@relates unique_ptr_ref **/
template <class T, class U>
inline bool operator>=(const unique_ptr_ref<T>& lhs,
                       const unique_ptr_ref<U>& rhs) {
  return !(lhs < rhs);
}
/** `nullptr` greater-or-equal test defined as `!(lhs < nullptr)`.
@relates unique_ptr_ref **/
template <class T>
inline bool operator>=(const unique_ptr_ref<T>& lhs, std::nullptr_t) {
  return !(lhs < nullptr);
}

/** `nullptr` greater-or-equal test defined as `!(nullptr < rhs)`.
@relates unique_ptr_ref **/
template <class T>
inline bool operator>=(std::nullptr_t, const unique_ptr_ref<T>& rhs) {
  return !(nullptr < rhs);
}

/** Pointer less-or-equal test defined as `!(rhs < lhs)` (note reversed
arguments).
@relates unique_ptr_ref **/
template <class T, class U>
inline bool operator<=(const unique_ptr_ref<T>& lhs,
                       const unique_ptr_ref<U>& rhs) {
  return !(rhs < lhs);
}
/** `nullptr` less-or-equal test defined as `!(nullptr < lhs)` (note reversed
arguments).
@relates unique_ptr_ref **/
template <class T>
inline bool operator<=(const unique_ptr_ref<T>& lhs, std::nullptr_t) {
  return !(nullptr < lhs);
}
/** `nullptr` less-or-equal test defined as `!(rhs < nullptr)` (note reversed
arguments).
@relates unique_ptr_ref **/
template <class T>
inline bool operator<=(std::nullptr_t, const unique_ptr_ref<T>& rhs) {
  return !(rhs < nullptr);
}

}  // namespace drake

//==============================================================================
//             std::hash<drake::unique_ptr_ref<T>> specialization
//==============================================================================
/* Adding this specialization to namespace std is permitted by the standard and
makes `std::unordered_map<unique_ptr_ref<T>>` work properly. See stackoverflow
discussion here: http://stackoverflow.com/questions/8157937/
  how-to-specialize-stdhashkeyoperator-for-user-defined-type-in-unordered
and cppreference: http://en.cppreference.com/w/cpp/utility/hash. */

namespace std {

/** Specializes `std::hash` for `drake::unique_ptr_ref<T>`. Note that this
specialization is added to the `std::` namespace. **/
template <class T>
class hash<::drake::unique_ptr_ref<T>> {
 public:
  /** Returns a hash of the contained pointer.
  @relates drake::unique_ptr_ref **/
  size_t operator()(const ::drake::unique_ptr_ref<T>& p) const {
    return hash<typename ::drake::unique_ptr_ref<T>::pointer>()(p.get());
  }
};

}  // namespace std
