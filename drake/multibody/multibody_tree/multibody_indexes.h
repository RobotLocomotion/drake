#pragma once

#include "drake/common/drake_assert.h"

#include <iostream>

template <class Derived>
class IndexBase {
 public:
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(IndexBase)
    IndexBase() : ix(InvalidIndex) {}
    explicit IndexBase(int i) : ix(i) {DRAKE_ASSERT(i >= 0 || i == InvalidIndex);}

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

    const Derived& operator++() {  /*prefix */
      DRAKE_ASSERT(is_valid()); ++ix; return get_derived();
    }
    Derived operator++(int) {  /*postfix*/
      DRAKE_ASSERT(is_valid()); ++ix; return Derived(ix-1);
    }

    const Derived& operator--() {  /*prefix */
      DRAKE_ASSERT(is_valid()); --ix; return get_derived();
    }
    Derived operator--(int) {  /*postfix*/
      DRAKE_ASSERT(is_valid()); --ix; return Derived(ix+1);
    }

    Derived& operator+=(int i) {
      DRAKE_ASSERT(is_valid());
      ix += i;
      return get_mutable_derived();
    }

    Derived& operator-=(int i) {
      DRAKE_ASSERT(is_valid());
      ix -= i;
      return get_mutable_derived();
    }

    static const Derived& Invalid() {
      static const Derived invalid;
      return invalid;
    }

 private:
  const Derived& get_derived() const {
    return *static_cast<const Derived*>(this);
  }

  Derived& get_mutable_derived() {
    return *static_cast<Derived*>(this);
  }

  int ix;
  static const int InvalidIndex{-1111111111};
};

template <class IndexType>
inline std::ostream& operator<<(
    std::ostream& o, const IndexBase<IndexType>& index) {
  if (index.is_invalid())
    o << "Invalid";
  else
    o << int(index);
  return o;
}

namespace drake {
namespace multibody {

class FrameIndex : public IndexBase<FrameIndex> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(FrameIndex)
  FrameIndex() = default;
  explicit FrameIndex(int i) : IndexBase<FrameIndex>(i) {}
};

class BodyIndex : public IndexBase<BodyIndex> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BodyIndex)
  BodyIndex() = default;
  explicit BodyIndex(int i) : IndexBase<BodyIndex>(i) {}
};

class MobilizerIndex : public IndexBase<MobilizerIndex> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MobilizerIndex)
  MobilizerIndex() = default;
  explicit MobilizerIndex(int i) : IndexBase<MobilizerIndex>(i) {}
};

/// For every MultibodyTree<T> the **world** body _always_ has this unique
/// identifier and it is always zero.
static const BodyIndex kWorldBodyId(0);

}  // namespace multibody
}  // namespace drake
