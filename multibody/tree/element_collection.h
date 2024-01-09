#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/ssize.h"
#include "drake/multibody/tree/string_view_map_key.h"

namespace drake {
namespace multibody {
namespace internal {

/* This models a vector<unique_ptr<Element<T>>>, plus the ability to perform
fast reverse lookups by name, and to remove elements but still efficiently
iterate over the non-removed elements.

@tparam_default_scalar
@tparam Element The MultibodyElement type, which must be one of:
  - Frame
  - Joint
  - JointActuator
  - ModelInstance
  - RigidBody
@tparam Index The corresponding index type for the given Element type. */
template <typename T, template <typename> class Element, typename Index>
class ElementCollection final {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ElementCollection);
  ElementCollection();
  ~ElementCollection();

  /* Returns the total number of (non-null) elements. */
  int num_elements() const { return ssize(indices_packed_); }

  /* Returns a read-only view of the (non-null) elements. The result only
  remains valid until the next call to any non-const member function.
  N.B. The return value is NOT indexed by the `Index` type. */
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
  could be either out-of-bounds, or in-bounds but a null element). */
  const Element<T>& get_element(Index index) const {
    if (!has_element(index)) {
      ThrowInvalidIndexException(typeid(Element<T>), index);
    }
    return *elements_by_index_[index];
  }

  /* Accesses an element via its index. Throws if the index is invalid (which
  could be either out-of-bounds, or in-bounds but a null element). */
  Element<T>& get_mutable_element(Index index) {
    if (!has_element(index)) {
      ThrowInvalidIndexException(typeid(Element<T>), index);
    }
    return *elements_by_index_[index];
  }

  /* Accesses an element via its index, without promising validity checking.
  @pre has_element(index) is true */
  const Element<T>& get_element_unchecked(Index index) const {
    DRAKE_ASSERT(has_element(index));
    return *elements_by_index_[index];
  }

  /* Returns a reference to the list of currently-valid indices. The result only
  remains valid until the next call to any non-const member function. */
  const std::vector<Index>& indices() const { return indices_packed_; }

  /* Returns a reference to the name lookup dictionary. The result only remains
  valid until the next call to any non-const member function. */
  const std::unordered_multimap<StringViewMapKey, Index>& names() const {
    return names_;
  }

  /* Returns the index that is one beyond the current maximum, i.e., the index
  that can be used for the next element passed to Add(). */
  Index next_index() const { return Index{ssize(elements_by_index_)}; }

  /* Adds a new element. If the element->index() equals the next_index(), then
  the element will be appended to the end (and next_index() will increment).
  Otherwise, the element->index() must refer to a currently-null element and the
  next_index() will not change. Returns a reference to the added Element, for
  convenience.
  @pre element != nullptr */
  Element<T>& Add(std::unique_ptr<Element<T>> element);

  /* Adds a new element, exactly like Add() except that the element pointer is
  borrowed from elsewhere, with no ownership transfer. */
  Element<T>& AddBorrowed(Element<T>* element);

  /* Removes an element (i.e., sets it to null).
  @pre has_element(index) is true */
  void Remove(Index index);

  /* Renames the given element. Throws if the index is invalid.
  This always calls `Element<T>::set_name(name)` on the element.
  @pre Element is a type that supports renaming (i.e., has `set_name`). */
  void Rename(Index index, std::string name);

  /* In support of MbT cloning, "adds" a null element at the current next_index,
  imitating the effect of calling both Add() and Remove() on that index. */
  void AddRemovedElementClone();

  /* In support of MbT cloning, appends nulls to this collection until it has
  the same indexing as `other`. (Shrinking is not allowed, only growing.)
  @pre next_index() <= other.next_index() */
  template <typename U>
  void ResizeToMatch(const ElementCollection<U, Element, Index>& other) {
    DRAKE_DEMAND(next_index() <= other.next_index());
    while (next_index() < other.next_index()) {
      AddRemovedElementClone();
    }
  }

 private:
  /* Throws an error for access to an out-of-bounds or removed element. */
  [[noreturn]] void ThrowInvalidIndexException(
      const std::type_info& element_type, Index index) const;

  /* The common implementation of both Add() and AddBorrowed(). */
  Element<T>& AddImpl(std::shared_ptr<Element<T>> maybe_owned);

  // The collection of elements, indexed by `Index`. Elements can be null (e.g.,
  // in the case of removed elements). Note that `shared_ptr` here is somewhat
  // misleading -- our public API doesn't have any concept of sharing; here it's
  // just a convenient way to handle pointers that are either owned or borrowed.
  std::vector<std::shared_ptr<Element<T>>> elements_by_index_;

  // Map used to find element indices by their actuator name. The same name
  // may appear in different model instances (hence the multimap).
  std::unordered_multimap<StringViewMapKey, Index> names_;

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
