#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

#include <iostream>

namespace drake {
namespace multibody {

/// A type-safe positive index that can be associated to a tag name. Different
/// instantiations of TaggedIndex are not interconvertible.
/// TaggedIndex allows for instantiations from an `int` as well as it allows to
/// convert back to an `int` via its conversion operator. Negative index values
/// are not allowed.
/// TaggedIndex can be used in places where an index is needed and therefore
/// basic operations such as prefix and postfix increment/decrement are
/// implemented. As an example, consider the creation of a tagged index
/// associated with class `Foo`. This can be done in code as: <pre>
///   using FooIndex = TaggedIndex<class FooTag>;
/// </pre>
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
  operator int() const { return index_;}

  /// @name Relational operators
  ///@{

  /// Equal to operator.
  bool operator==(int  i) const { return index_ == i;}

  /// Not equal to operator.
  bool operator!=(int  i) const { return !operator==(i);}

  /// Less than operator.
  bool operator< (int  i) const { return index_ < i;}

  /// Greater than or equal to operator.
  bool operator>=(int  i) const { return !operator<(i);}

  /// Greater than operator.
  bool operator> (int  i) const { return index_ > i;}

  /// Less than or equal to operator.
  bool operator<=(int  i) const { return !operator>(i);}
  ///@}

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
    return TaggedIndex(index_-1);
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
    return TaggedIndex(index_+1);
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

/// Stream insertion operator to write a TaggedIndex into a std::ostream.
template <class Tag>
inline std::ostream& operator<<(
    std::ostream& o, const TaggedIndex<Tag>& index) {
  o << int(index);
  return o;
}

using FrameIndex = TaggedIndex<class FrameTag>;
using BodyIndex = TaggedIndex<class BodyTag>;
using MobilizerIndex = TaggedIndex<class MobilizerTag>;

/// For every MultibodyTree<T> the **world** body _always_ has this unique
/// identifier and it is always zero.
static const BodyIndex kWorldBodyId(0);

}  // namespace multibody
}  // namespace drake
