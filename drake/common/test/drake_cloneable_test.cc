#include "drake/common/drake_cloneable.h"

#include <memory>

#include <gtest/gtest.h>

// This provides concrete examples of the `is_cloneable` function
// for showing which classes are considered cloneable and which
// aren't.
namespace drake {
namespace {

using std::make_unique;
using std::unique_ptr;

//-----  A Collection of Dummy Classes for tests

// A fully copyable class (has both copy constructor and Clone). Also used to
// test inheritance of Clone methods.
// Cloneable!
class Base {
 public:
  Base() {}
  Base(const Base& b) = default;
  unique_ptr<Base> Clone() const { return make_unique<Base>(); }
};

// An inherited class with no copy constructor, which inherits the parent's
// Clone method (but doesn't return the appropriate type).
// NOT CLONEABLE!
class InheritClone : public Base {
 public:
  InheritClone() {}
  InheritClone(const InheritClone& c) = delete;
};

// A class with no copy constructor and a Clone method with the wrong return
// type -- no explicit conversion from * to unique_ptr.
// NOT CLONEABLE!
class BadClone : public Base {
 public:
  BadClone() {}
  BadClone(const BadClone& c) = delete;
  BadClone* Clone() const { return new BadClone(); }
};

// A class with no copy constructor and a non-const Clone method.
// NOT CLONEABLE!
class NonConstClone : public Base {
 public:
  NonConstClone() {}
  NonConstClone(const NonConstClone& c) = delete;
  unique_ptr<NonConstClone> Clone() { return make_unique<NonConstClone>(); }
};

// Class with no copy semantics (no copy constructor, no Clone)
// NOT CLONEABLE!
class CantCopy {
 public:
  CantCopy() {}
  CantCopy(const CantCopy& f) = delete;
};

// A class with a malformed Clone method, but it has a copy constructor.
// NOT CLONEABLE! (But copyable.)
class BadCloneCopy {
 public:
  BadCloneCopy* Clone() const { return new BadCloneCopy(); }
};

// A class with a copy constructor but no clone method.
// NOT CLONEABLE! (But copyable.)
class NoClone {
 public:
  NoClone() {}
  NoClone(const NoClone& f) {}
};

// A class with no copy constructor, but a valid Clone method.
// Cloneable!
class CloneOnly {
 public:
  CloneOnly() {}
  CloneOnly(const CloneOnly& f) = delete;
  unique_ptr<CloneOnly> Clone() const { return make_unique<CloneOnly>(); }
};

GTEST_TEST(IsCloneableTest, FullyCopyable) {
  EXPECT_TRUE(is_cloneable<Base>::value);
}

GTEST_TEST(IsCloneableTest, OnlyCloneable) {
  EXPECT_TRUE(is_cloneable<CloneOnly>::value);
}

GTEST_TEST(IsCloneableTest, InheritCloneFail) {
  EXPECT_FALSE(is_cloneable<InheritClone>::value);
}

GTEST_TEST(IsCloneableTest, BadCloneReturnFail) {
  EXPECT_FALSE(is_cloneable<BadClone>::value);
}

GTEST_TEST(IsCloneableTest, NonConstCloneFail) {
  EXPECT_FALSE(is_cloneable<NonConstClone>::value);
}

GTEST_TEST(IsCloneableTest, UnCopyableFail) {
  EXPECT_FALSE(is_cloneable<CantCopy>::value);
}

GTEST_TEST(IsCloneableTest, CopyableWithBadCloneFail) {
  EXPECT_FALSE(is_cloneable<BadCloneCopy>::value);
}

GTEST_TEST(IsCloneableTest, CopyableButNoCloneFail) {
  EXPECT_FALSE(is_cloneable<NoClone>::value);
}

}  // namespace
}  // namespace drake
