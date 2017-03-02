#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

#include <iostream>

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
  explicit TaggedIndex(int index) : ix(index) {
    DRAKE_ASSERT(ix >= 0);
  }

  /// Converstion to int operator.
  operator int() const { return ix;}

  /// @name Relational operators
  ///@{

  /// Equal to operator.
  bool operator==(int  i) const { return ix == i;}

  /// Not equal to operator.
  bool operator!=(int  i) const { return !operator==(i);}

  /// Less than operator.
  bool operator< (int  i) const { return ix < i;}

  /// Greater than or equal to operator.
  bool operator>=(int  i) const { return !operator<(i);}

  /// Greater than operator.
  bool operator> (int  i) const { return ix > i;}

  /// Less than or equal to operator.
  bool operator<=(int  i) const { return !operator>(i);}
  ///@}

  /// @name Arithmetic operators.
  ///@{

  /// Prefix increment operator.
  const TaggedIndex& operator++() {
    ++ix;
    return *this;
  }

  /// Postfix increment operator.
  TaggedIndex operator++(int) {
    ++ix;
    return TaggedIndex(ix-1);
  }

  /// Prefix decrement operator.
  /// In Debug builds this method asserts that the resulting index is
  /// non-negative.
  const TaggedIndex& operator--() {
    --ix;
    DRAKE_ASSERT(ix >= 0);
    return *this;
  }

  /// Postfix decrement operator.
  /// In Debug builds this method asserts that the resulting index is
  /// non-negative.
  TaggedIndex operator--(int) {
    --ix;
    DRAKE_ASSERT(ix >= 0);
    return TaggedIndex(ix+1);
  }
  ///@}

  /// @name Compound assignment operators.
  ///@{

  /// Addition assignment operator.
  TaggedIndex& operator+=(int i) {
    ix += i;
    return *this;
  }

  /// Subtraction assignment operator.
  /// In Debug builds this method asserts that the resulting index is
  /// non-negative.
  TaggedIndex& operator-=(int i) {
    ix -= i;
    DRAKE_ASSERT(ix >= 0);
    return *this;
  }
  ///@}

 private:
  int ix{0};
};

/// Stream insertion operator to write a TaggedIndex into a std::ostream.
template <class Tag>
inline std::ostream& operator<<(
    std::ostream& o, const TaggedIndex<Tag>& index) {
  o << int(index);
  return o;
}

namespace drake {
namespace multibody {

using FrameIndex = TaggedIndex<class FrameTag>;
using BodyIndex = TaggedIndex<class BodyTag>;
using MobilizerIndex = TaggedIndex<class MobilizerTag>;

/// For every MultibodyTree<T> the **world** body _always_ has this unique
/// identifier and it is always zero.
static const BodyIndex kWorldBodyId(0);

}  // namespace multibody
}  // namespace drake
