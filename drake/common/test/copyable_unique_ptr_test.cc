#include "drake/common/copyable_unique_ptr.h"

#include <memory>

#include <gtest/gtest.h>

namespace drake {
namespace  {

using std::make_unique;
using std::unique_ptr;

//--------- A Collection of Dummy Classes for test

template <typename T>
using cup = copyable_unique_ptr<T>;

// Indicator of where a particular instance came from.
enum class Origin {
  UNINITIZLIZED,
  CONSTRUCT,
  COPY,
  CLONE
};

// A simple base class that stores a value and a construction origin. This only
// has a *protected* copy constructor. Child classes can invoke it, but it
// won't be considered copyable.
struct Base {
  Base(int v, Origin org=Origin::CONSTRUCT) : origin(org), value(v) {}

  Origin origin{Origin::UNINITIZLIZED};
  int value{-1};
 protected:
  Base(const Base& b) : origin(Origin::COPY), value(b.value) {}
};


// A class that is copyable, but has no Clone method.
struct CopyOnly : Base {
  CopyOnly(int v, Origin org=Origin::CONSTRUCT) : Base(v, org) {}
  CopyOnly(const CopyOnly& b) : Base(b) {}
};

// Confirms that a class with only a public copy constructor is considered
// copyable and copies appropriately.
GTEST_TEST(CopyableUniquePtrTest, CopyOnlySuccess) {
  cup<CopyOnly> ptr(new CopyOnly(1));
  EXPECT_EQ(ptr->origin, Origin::CONSTRUCT);
  cup<CopyOnly> copy(ptr);
  EXPECT_EQ(copy->origin, Origin::COPY);
  EXPECT_EQ(copy->value, ptr->value);
  copy->value++;
  EXPECT_NE(copy->value, ptr->value);
}


// A fully copyable class (has both copy constructor and Clone). Confirms that
// in the presence of both, the copy constructor is preferred.
struct FullyCopyable : Base {
  FullyCopyable(int v, Origin org=Origin::CONSTRUCT) : Base(v, org) {}

  FullyCopyable(const FullyCopyable& b) : Base(b) {}
  unique_ptr<FullyCopyable> Clone() const {
    return make_unique<FullyCopyable>(value, Origin::CLONE);
  }
};

// Confirms that the copy constructor is preferred when both exist.
GTEST_TEST(CopyableUniquePtrTest, FullyCopyableSuccess) {
  cup<FullyCopyable> ptr(new FullyCopyable(1));
  EXPECT_EQ(ptr->origin, Origin::CONSTRUCT);
  cup<FullyCopyable> copy(ptr);
  EXPECT_EQ(copy->origin, Origin::COPY);
  EXPECT_EQ(copy->value, ptr->value);
  copy->value++;
  EXPECT_NE(copy->value, ptr->value);
}

// A class with only a Clone method.
struct CloneOnly : Base {
  CloneOnly(int v, Origin org=Origin::CONSTRUCT) : Base(v, org) {}
  CloneOnly(const CloneOnly&) = delete;
  unique_ptr<CloneOnly> Clone() const {
    return make_unique<CloneOnly>(value, Origin::CLONE);
  }
};

// Confirms that a class that has a proper implementation of Clone, but no
// copy constructor clones itself.
GTEST_TEST(CopyableUniquePtrTest, CloneOnlySuccess) {
  cup<CloneOnly> ptr(new CloneOnly(1));
  EXPECT_EQ(ptr->origin, Origin::CONSTRUCT);
  cup<CloneOnly> copy(ptr);
  EXPECT_EQ(copy->origin, Origin::CLONE);
  EXPECT_EQ(copy->value, ptr->value);
  copy->value++;
  EXPECT_NE(copy->value, ptr->value);
}

// TODO(SeanCurtis-TRI): Is this test meaningful?
/*
struct FullChild : Base {
  FullChild(int u, int v, Origin org=Origin::CONSTRUCT) : Base(v, org), unique_value(u) {}

  FullChild(const FullChild& b) : Base(b), unique_value(b.unique_value) {}
  unique_ptr<FullChild> Clone() const {
    return make_unique<FullChild>(unique_value, value, Origin::CLONE);
  }
  int unique_value{-1};
};

// Confirms that a class that has a proper implementation of Clone, but no
// copy constructor clones itself.
GTEST_TEST(CopyableUniquePtrTest, CloneOnlySuccess) {
  cup<CloneOnly> ptr(new CloneOnly(1));
  EXPECT_EQ(ptr->origin, Origin::CONSTRUCT);
  cup<CloneOnly> copy(ptr);
  EXPECT_EQ(copy->origin, Origin::CLONE);
  EXPECT_EQ(copy->value, ptr->value);
  copy->value++;
  EXPECT_NE(copy->value, ptr->value);
}
 */

// This class isn't copyable at all -- no copy constructor and no Clone method.
struct NotCopyable {
  NotCopyable() {}
  NotCopyable(const NotCopyable& c) = delete;
};

GTEST_TEST(CopyableUniquePtrTest, UnCopyableFailure) {
// TODO(SeanCurtis-TRI): This is an error, I shouldn't even be able to
// instantiate such a thing.
//  cup<NotCopyable> ptr(new NotCopyable());
// TODO(SeanCurtis-TRI): Figure out how to make this compilation error testable.
//  cup<NotCopyable> copy(ptr);
}

}  // namespace
}  // namespace drake
