#pragma once

#include "drake/common/drake_assert.h"

#include <iostream>

#define DrakeMultibody_DEFINE_INDEX_TYPE(NAME)   \
class NAME {                         \
    int ix;                                 \
    static const int InvalidIndex{-1111111111};  \
public:                                     \
    NAME() : ix(InvalidIndex) { }       \
    explicit NAME(int i) : ix(i)      {DRAKE_ASSERT(i>=0 || i==InvalidIndex);} \
    operator int() const {return ix;}               \
    bool is_valid() const {return ix>=0;}            \
    bool is_invalid() const {return ix == InvalidIndex;}            \
    void invalidate(){clear();}                     \
    void clear(){ix=InvalidIndex;}           \
    \
    bool operator==(int  i) const { return ix==i;}    \
    bool operator!=(int  i) const { return !operator==(i);}    \
    \
    bool operator< (int  i) const { return ix<i;}        \
    bool operator>=(int  i) const {return !operator<(i);}    \
    \
    bool operator> (int  i) const { return ix>i;}        \
    bool operator<=(int  i) const {return !operator>(i);}    \
    \
    const NAME& operator++() {DRAKE_ASSERT(is_valid()); ++ix; return *this;}       /*prefix */   \
    NAME operator++(int)     {DRAKE_ASSERT(is_valid()); ++ix; return NAME(ix-1);}  /*postfix*/   \
    const NAME& operator--() {DRAKE_ASSERT(is_valid()); --ix; return *this;}       /*prefix */   \
    NAME operator--(int)     {DRAKE_ASSERT(is_valid()); --ix; return NAME(ix+1);}  /*postfix*/   \
    NAME next() const {DRAKE_ASSERT(is_valid()); return NAME(ix+1);}                             \
    NAME prev() const {DRAKE_ASSERT(is_valid()); return NAME(ix-1);} /*might return -1*/         \
    \
    NAME& operator+=(int i)  {DRAKE_ASSERT(is_valid()); ix+=i; return *this;}     \
    NAME& operator-=(int i)  {DRAKE_ASSERT(is_valid()); ix-=i; return *this;}     \
    \
    static const NAME& Invalid() {static const NAME invalid; return invalid;}       \
    static bool isValid(int  i) {return i>=0;}                                      \
};  \
    \
inline std::ostream& operator<<(std::ostream& o, const NAME& index) {          \
      if (index.is_invalid())                                                  \
        o << "Invalid";                                                        \
      else                                                                     \
        o << int(index);                                                       \
      return o;                                                                \
    }                                                                          \

namespace drake {
namespace multibody {
DrakeMultibody_DEFINE_INDEX_TYPE(FrameIndex);
DrakeMultibody_DEFINE_INDEX_TYPE(BodyIndex);
DrakeMultibody_DEFINE_INDEX_TYPE(BodyNodeIndex);
DrakeMultibody_DEFINE_INDEX_TYPE(MobilizerIndex);

/// For every MultibodyTree<T> the *world* body _always_ has this unique
/// identifier and it is always zero.
static const BodyIndex kWorldBodyId(0);

}  // namespace multibody
}  // namespace drake
