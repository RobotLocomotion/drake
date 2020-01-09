#include "drake/common/is_cloneable.h"

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

GTEST_TEST(IsCloneableTest, FullyCopyable) {
  EXPECT_TRUE(is_cloneable<Base>::value);
}

// A class with no copy constructor, but a valid Clone method.
// Cloneable!
class CloneOnly {
 public:
  CloneOnly() {}
  CloneOnly(const CloneOnly& f) = delete;
  unique_ptr<CloneOnly> Clone() const { return make_unique<CloneOnly>(); }
};

GTEST_TEST(IsCloneableTest, OnlyCloneable) {
  EXPECT_TRUE(is_cloneable<CloneOnly>::value);
}

// Confirms that a const type is still considered cloneable if the type is
// cloneable.
GTEST_TEST(IsCloneableTest, TestConstType) {
  EXPECT_TRUE(is_cloneable<const CloneOnly>::value);
}

// A class with no copy constructor, but has a Clone that returns the parent
// class pointer. The parent is representative of *any* ancestor in the
// inheritance tree.
// Cloneable!
class CloneToParent : public Base {
 public:
  CloneToParent() {}
  CloneToParent(const CloneToParent& c) = delete;
  unique_ptr<Base> Clone() const {
    return make_unique<CloneToParent>();
  }
};

// This tests the cloneability of a class that *does* have a const Clone()
// method, but the return type is a parent class type. This is not cloneable
// because it would amount to:
//   Base* base_ptr = nullptr;
//   Derived* ptr = base_ptr;
// which is clearly invalid.
GTEST_TEST(IsCloneableTest, CloneUsingInheritedMethod) {
  EXPECT_FALSE(is_cloneable<CloneToParent>::value);
}

// A class with no copy constructor and a Clone method with the wrong return
// type -- no explicit conversion from * to unique_ptr.
// NOT CLONEABLE!
class BadClone : public Base {
 public:
  BadClone() {}
  BadClone(const BadClone& c) = delete;
  [[nodiscard]] BadClone* Clone() const { return new BadClone(); }
};

GTEST_TEST(IsCloneableTest, BadCloneReturnFail) {
  EXPECT_FALSE(is_cloneable<BadClone>::value);
}

// A class with no copy constructor and a non-const Clone method.
// NOT CLONEABLE!
class NonConstClone : public Base {
 public:
  NonConstClone() {}
  NonConstClone(const NonConstClone& c) = delete;
  unique_ptr<NonConstClone> Clone() { return make_unique<NonConstClone>(); }
};

GTEST_TEST(IsCloneableTest, NonConstCloneFail) {
  EXPECT_FALSE(is_cloneable<NonConstClone>::value);
}

// Class with no copy semantics (no copy constructor, no Clone)
// NOT CLONEABLE!
class CantCopy {
 public:
  CantCopy() {}
  CantCopy(const CantCopy& f) = delete;
};

GTEST_TEST(IsCloneableTest, UnCopyableFail) {
  EXPECT_FALSE(is_cloneable<CantCopy>::value);
}

// A class with a malformed Clone method, but it has a copy constructor.
// NOT CLONEABLE! (But copyable.)
class BadCloneCopy {
 public:
  BadCloneCopy() {}
  BadCloneCopy(const BadCloneCopy& f) = default;
  [[nodiscard]] BadCloneCopy* Clone() const { return new BadCloneCopy(); }
};

GTEST_TEST(IsCloneableTest, CopyableWithBadCloneFail) {
  EXPECT_FALSE(is_cloneable<BadCloneCopy>::value);
}

// A class with a copy constructor but no clone method.
// NOT CLONEABLE! (But copyable.)
class NoClone {
 public:
  NoClone() {}
  NoClone(const NoClone& f) {}
};

GTEST_TEST(IsCloneableTest, CopyableButNoCloneFail) {
  EXPECT_FALSE(is_cloneable<NoClone>::value);
}

}  // namespace
}  // namespace drake
