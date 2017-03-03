#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

#include <iostream>

namespace drake {
namespace multibody {

/// A type-safe non-negative index that can be associated to a tag name.
/// Different instantiations of TaggedIndex are not interconvertible.
/// TaggedIndex allows for instantiations from an `int` as well as it allows to
/// convert back to an `int` via its conversion operator. Negative index values
/// are not allowed.
/// TaggedIndex can be used in places where an index is needed and therefore
/// basic operations such as prefix and postfix increment/decrement are
/// implemented. As an example, consider the creation of a tagged index
/// associated with class `Foo`. This can be done in code as: <pre>
///   using FooIndex = TaggedIndex<class FooTag>;
/// </pre>
/// where the class `FooTag` above should simply be a dummy argument, a
/// never-defined type.
///
/// @tparam Tag The name of the tag associated with a class type.
template <class Tag>
class TaggedIndex {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TaggedIndex)

  /// Default constructor intializes `this` to the zero index.
  TaggedIndex() {}

  /// Construction from an `int` value.
  /// For Debug builds this constructor aborts if the provided input `int`
  /// `index` is negative. There is no check for Release builds.
  explicit TaggedIndex(int index) : index_(index) {
    DRAKE_ASSERT(index_ >= 0);
  }

  /// Converstion to int operator.
  operator int() const { return index_; }

  /// @name Arithmetic operators.
  ///@{

  /// Prefix increment operator.
  const TaggedIndex& operator++() {
    ++index_;
    return *this;
  }

  /// Postfix increment operator.
  TaggedIndex operator++(int) {
    ++index_;
    return TaggedIndex(index_ - 1);
  }

  /// Prefix decrement operator.
  /// In Debug builds this method asserts that the resulting index is
  /// non-negative.
  const TaggedIndex& operator--() {
    --index_;
    DRAKE_ASSERT(index_ >= 0);
    return *this;
  }

  /// Postfix decrement operator.
  /// In Debug builds this method asserts that the resulting index is
  /// non-negative.
  TaggedIndex operator--(int) {
    --index_;
    DRAKE_ASSERT(index_ >= 0);
    return TaggedIndex(index_ + 1);
  }
  ///@}

  /// @name Compound assignment operators.
  ///@{

  /// Addition assignment operator.
  TaggedIndex& operator+=(int i) {
    index_ += i;
    return *this;
  }

  /// Subtraction assignment operator.
  /// In Debug builds this method asserts that the resulting index is
  /// non-negative.
  TaggedIndex& operator-=(int i) {
    index_ -= i;
    DRAKE_ASSERT(index_ >= 0);
    return *this;
  }
  ///@}

 private:
  int index_{0};
};

/// Type used to identify frames by index in a multibody tree system.
using FrameIndex = TaggedIndex<class FrameTag>;

/// Type used to identify bodies by index in a multibody tree system.
using BodyIndex = TaggedIndex<class BodyTag>;

/// Type used to identify mobilizers by index in a multibody tree system.
using MobilizerIndex = TaggedIndex<class MobilizerTag>;

/// For every MultibodyTree<T> the **world** body _always_ has this unique
/// identifier and it is always zero.
// Note:
//   static global variables are strongly discouraged by the C++ style guide:
// https://google.github.io/styleguide/cppguide.html#Static_and_Global_Variables
BodyIndex world_id() { return BodyIndex(0); }

}  // namespace multibody
}  // namespace drake
