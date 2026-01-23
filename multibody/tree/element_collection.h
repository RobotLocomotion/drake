#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/string_unordered_map.h"

namespace drake {
namespace multibody {
namespace internal {

/* This models a collection of `Element<T>`s (e.g., something like a
`vector<unique_ptr<Element<T>>>`) but with the following variations compared to
a plain vector:
- the ability to perform fast O(1) lookups by `element->name()`;
- the ability to null out elements but still efficiently iterate over the
  non-null elements (either by pointer or by index);
- the ability to store both owned and un-owned pointers.

This helper class is used by MultibodyTree to maintain its collections of
elements that match the above description. (See the `@tparam` below for the
full list of element types.)

Note that multiple elements are allowed to have the same `element->name()`. For
example, two `RigidBody`s that are owned by two different model instances can
have the same `name()` but will have different `scoped_name()`s.

Currently, the only MultibodyTree element type that sometimes uses un-owned
pointers is Frame -- when the Frame is a RigidBodyFrame, then it is owned by the
RigidBody and the RigidBody is owned by the MultibodyTree. Note that other Frame
types like FixedOffsetFrame *are* directly owned by this collection.

@tparam_default_scalar
@tparam Element The MultibodyElement type, which must be one of:
  - Frame
  - Joint
  - JointActuator
  - ModelInstance
  - RigidBody
  - DeformableBody
@tparam Index The corresponding index type for the given Element type. */
template <typename T, template <typename> class Element, typename Index>
class ElementCollection final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ElementCollection);
  ElementCollection();
  ~ElementCollection();

  /* Returns the total number of (non-null) elements. */
  int num_elements() const { return ssize(indices_packed_); }

  /* Returns a read-only view of the (non-null) elements. The result is only
  guaranteed to remain valid until the next call to any non-const member
  function.
  @warning The return value is NOT indexed by the `Index` type. */
  const std::vector<const Element<T>*>& elements() const {
    return elements_packed_;
  }

  /* Returns true iff the given index refers to a valid (non-null) element.
  An out-of-bounds index simply returns false; it does not throw. */
  bool has_element(Index index) const {
    return index.is_valid() && (index < ssize(elements_by_index_)) &&
           (elements_by_index_[index] != nullptr);
  }

  /* Accesses an element via its index. Throws if the index is invalid (which
  could be either out-of-bounds, or in-bounds but a null element).
  See get_element_unchecked() to skip the validity checking. */
  const Element<T>& get_element(Index index) const {
    if (!has_element(index)) {
      ThrowInvalidIndexException(index);
    }
    return *elements_by_index_[index];
  }

  /* Accesses an element via its index. Throws if the index is invalid (which
  could be either out-of-bounds, or in-bounds but a null element). */
  Element<T>& get_mutable_element(Index index) {
    if (!has_element(index)) {
      ThrowInvalidIndexException(index);
    }
    return *elements_by_index_[index];
  }

  /* Accesses an element via its index, without promising validity checking.
  @pre has_element(index) is true */
  const Element<T>& get_element_unchecked(Index index) const {
    DRAKE_ASSERT(has_element(index));
    return *elements_by_index_[index];
  }

  /* Returns a reference to the list of currently-valid indices. The result is
  only guaranteed to remain valid until the next call to any non-const member
  function. */
  const std::vector<Index>& indices() const { return indices_packed_; }

  /* Returns a reference to the name lookup dictionary. The result is only
  guaranteed to remain valid until the next call to any non-const member
  function. */
  const string_unordered_multimap<Index>& names_map() const {
    return names_map_;
  }

  /* Returns the index that is one beyond the current maximum, i.e., the index
  that can be used for the next element passed to Add(). */
  Index next_index() const { return Index{ssize(elements_by_index_)}; }

  /* Adds a new element. If the element->index() equals the next_index(), then
  the element will be appended to the end (and next_index() will increment).
  Otherwise, the element->index() must refer to a currently-null element and the
  next_index() will not change. Returns a reference to the added Element, for
  convenience.
  @pre element != nullptr
  @pre has_element(element->index()) is false */
  Element<T>& Add(std::unique_ptr<Element<T>> element);

  /* Adds a new element, exactly like Add() except that the element pointer is
  borrowed from elsewhere, with no ownership transfer. */
  Element<T>& AddBorrowed(Element<T>* element);

  /* Adds a null element at next_index(). */
  void AppendNull();

  /* Removes an element (i.e., sets it to null).
  @pre has_element(index) is true */
  void Remove(Index index);

  /* Renames the given element. Throws if the index is invalid.
  This always calls `Element<T>::set_name(name)` on the element.
  @pre Element is a type that supports renaming (i.e., has `set_name`). */
  void Rename(Index index, std::string name);

  /* In support of MbT cloning, appends nulls to this collection until it has
  the same indexing as `other`. (Shrinking is not allowed, only growing.)
  @pre next_index() <= other.next_index() */
  template <typename U>
  void ResizeToMatch(const ElementCollection<U, Element, Index>& other) {
    DRAKE_DEMAND(next_index() <= other.next_index());
    while (next_index() < other.next_index()) {
      AppendNull();
    }
  }

 private:
  /* Throws an error for access to an out-of-bounds or removed element. */
  [[noreturn]] void ThrowInvalidIndexException(Index index) const;

  /* The common implementation of both Add() and AddBorrowed(). */
  Element<T>& AddImpl(std::shared_ptr<Element<T>> maybe_owned);

  /* Given a (name, index) pair, returns the iterator into `names_map_` that
  matches those values, or else `names_map_.end()` when no match is found. */
  typename string_unordered_multimap<Index>::const_iterator FindNamesIterator(
      std::string_view name, Index index) const;

  // The collection of elements, indexed by `Index`. Elements can be null (e.g.,
  // in the case of removed elements). Note that `shared_ptr` here is somewhat
  // misleading -- our public API doesn't have any concept of sharing; here it's
  // just a convenient way to handle pointers that are either owned or borrowed.
  std::vector<std::shared_ptr<Element<T>>> elements_by_index_;

  // Map used to find element indices by their actuator name. The same name
  // may appear in different model instances (hence the multimap).
  string_unordered_multimap<Index> names_map_;

  // The elements of `elements_by_index_` that have _not_ been removed.
  // N.B. This vector is NOT indexed by `Index`.
  std::vector<const Element<T>*> elements_packed_;

  // The indices of `elements_packed_`.
  // N.B. This vector is NOT indexed by `Index`.
  std::vector<Index> indices_packed_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake
