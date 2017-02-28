#pragma once

#include "drake/common/drake_assert.h"

#include <iostream>

template <class Tag>
class TaggedInt {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TaggedInt)
  TaggedInt() : ix(InvalidIndex) {}
  explicit TaggedInt(int i) : ix(i) {DRAKE_ASSERT(i >= 0 || i == InvalidIndex);}

  operator int() const { return ix;}
  bool is_valid() const { return ix >= 0;}
  bool is_invalid() const { return ix == InvalidIndex;}
  void invalidate() { ix = InvalidIndex;}

  bool operator==(int  i) const { return ix == i;}
  bool operator!=(int  i) const { return !operator==(i);}

  bool operator< (int  i) const { return ix < i;}
  bool operator>=(int  i) const { return !operator<(i);}

  bool operator> (int  i) const { return ix > i;}
  bool operator<=(int  i) const { return !operator>(i);}

  const TaggedInt& operator++() {  /*prefix */
    DRAKE_ASSERT(is_valid());
    ++ix;
    return *this;
  }
  TaggedInt operator++(int) {  /*postfix*/
    DRAKE_ASSERT(is_valid()); ++ix; return TaggedInt(ix-1);
  }

  const TaggedInt& operator--() {  /*prefix */
    DRAKE_ASSERT(is_valid()); --ix; return *this;
  }
  TaggedInt operator--(int) {  /*postfix*/
    DRAKE_ASSERT(is_valid()); --ix; return TaggedInt(ix+1);
  }

  TaggedInt& operator+=(int i) {
    DRAKE_ASSERT(is_valid());
    ix += i;
    return *this;
  }

  TaggedInt& operator-=(int i) {
    DRAKE_ASSERT(is_valid());
    ix -= i;
    return *this;
  }

  static const TaggedInt& Invalid() {
    static const TaggedInt invalid;
    return invalid;
  }

 private:
  int ix;
  static const int InvalidIndex{-1111111111};
};

template <class Tag>
inline std::ostream& operator<<(
    std::ostream& o, const TaggedInt<Tag>& index) {
  if (index.is_invalid())
    o << "Invalid";
  else
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
