#pragma once

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"

#include <iostream>

/// A type-safe positive index that can be associated to a tag name. Different
/// instantiations of TaggedInt are not interconvertible.
/// TaggedInt allows for instantiations from an `int` as well as it allows to
/// convert back to an `int` via its conversion operator. Negative index values
/// are not allowed.
/// TaggedInt can be used in places where an index is needed and therefore basic
/// operations such as prefix and postfix increment/decrement are implemented.
/// As an example, consider the creation of a tagged index associated with class
/// `Foo`. This can be done in code as: <pre>
///   using FooIndex = TaggedInt<class FooTag>;
/// </pre>
///
/// @tparam Tag The name of the tag associated with a class type.
template <class Tag>
class TaggedInt {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TaggedInt)

  /// Default constructor intializes `this` to the zero index.
  TaggedInt() {}

  /// Construction from an `int` value.
  /// For Debug builds this constructor aborts if the provided input `int`
  /// `index` is negative. There is no check for Release builds.
  explicit TaggedInt(int index) : ix(index) {
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

  /// Greather than or equal to operator.
  bool operator>=(int  i) const { return !operator<(i);}

  /// Greather than operator.
  bool operator> (int  i) const { return ix > i;}

  /// Less than or equal to operator.
  bool operator<=(int  i) const { return !operator>(i);}
  ///@}

  /// @name Arithmetic operators.
  ///@{

  /// Prefix increment operator.
  const TaggedInt& operator++() {
    ++ix;
    return *this;
  }

  /// Postfix increment operator.
  TaggedInt operator++(int) {
    ++ix;
    return TaggedInt(ix-1);
  }

  /// Prefix decrement operator.
  /// In Debug builds this method asserts that the resulting index is
  /// non-negative.
  const TaggedInt& operator--() {
    --ix;
    DRAKE_ASSERT(ix >= 0);
    return *this;
  }

  /// Postfix decrement operator.
  /// In Debug builds this method asserts that the resulting index is
  /// non-negative.
  TaggedInt operator--(int) {
    --ix;
    DRAKE_ASSERT(ix >= 0);
    return TaggedInt(ix+1);
  }
  ///@}

  /// @name Compound assignment operators.
  ///@{

  /// Addition assignment operator.
  TaggedInt& operator+=(int i) {
    ix += i;
    return *this;
  }

  /// Subtraction assignment operator.
  /// In Debug builds this method asserts that the resulting index is
  /// non-negative.
  TaggedInt& operator-=(int i) {
    ix -= i;
    DRAKE_ASSERT(ix >= 0);
    return *this;
  }
  ///@}

 private:
  int ix{0};
};

/// Stream insertion operator to write a TaggedInt into a std::ostream.
template <class Tag>
inline std::ostream& operator<<(
    std::ostream& o, const TaggedInt<Tag>& index) {
  o << int(index);
  return o;
}

namespace drake {
namespace multibody {

using FrameIndex = TaggedInt<class FrameTag>;
using BodyIndex = TaggedInt<class BodyTag>;
using MobilizerIndex = TaggedInt<class MobilizerTag>;

/// For every MultibodyTree<T> the **world** body _always_ has this unique
/// identifier and it is always zero.
static const BodyIndex kWorldBodyId(0);

}  // namespace multibody
}  // namespace drake
