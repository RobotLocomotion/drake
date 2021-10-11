#include "drake/common/copyable_unique_ptr.h"

#include <memory>
#include <regex>
#include <sstream>

#include <gtest/gtest.h>

#include "drake/common/is_cloneable.h"
#include "drake/common/test_utilities/is_dynamic_castable.h"
#include "drake/common/unused.h"

namespace drake {
namespace  {

using std::make_unique;
using std::move;
using std::regex;
using std::regex_match;
using std::stringstream;
using std::unique_ptr;

// Convenience alias to keep the test code lines shorter.
template <typename T>
using cup = copyable_unique_ptr<T>;

// -------------------- copy functionality tests ------------------------
// These tests cover the actual copyable semantics. Confirming that the
// protocol selecting between Clone and copy constructor is consistent with the
// definition and that the implementation invokes the correct copy mechanism
// and, when neither exists, recognizes an uncopyable class.
// These tests implicitly test copyable_unique_ptr's copy constructor.

// Indicator of where a particular instance came from.
enum class Origin {
  UNINITIALIZED,
  CONSTRUCT,
  COPY,
  CLONE
};

// A simple base class that stores a value and a construction origin. This only
// has a *protected* copy constructor. Child classes can invoke it, but it
// won't be considered copyable.
struct Base {
  explicit Base(int v, Origin org = Origin::CONSTRUCT)
      : origin(org), value(v) {}
  virtual ~Base() {}

  Origin origin{Origin::UNINITIALIZED};
  int value{-1};
 protected:
  Base(const Base& b) : origin(Origin::COPY), value(b.value) {}
};

// A class that is copyable, but has no Clone method.
struct CopyOnly : Base {
  explicit CopyOnly(int v, Origin org = Origin::CONSTRUCT) : Base(v, org) {}
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
  ++copy->value;
  EXPECT_NE(copy->value, ptr->value);
}

// This class has a private copy constructor but declares copyable_unique_ptr
// to be a friend so it should still be copyable.
class FriendsWithBenefitsCopy {
 public:
  explicit FriendsWithBenefitsCopy(int value) : value_(value) {}
  int value() const {return value_;}

 private:
  friend class copyable_unique_ptr<FriendsWithBenefitsCopy>;
  FriendsWithBenefitsCopy(const FriendsWithBenefitsCopy&) = default;

  int value_{0};
};

// This class has no copy constructor and a private Clone() method but declares
// copyable_unique_ptr to be a friend so it should still be copyable.
class FriendsWithBenefitsClone {
 public:
  explicit FriendsWithBenefitsClone(int value) : value_(value) {}
  int value() const {return value_;}
  FriendsWithBenefitsClone(const FriendsWithBenefitsClone&) = delete;

 private:
  friend class copyable_unique_ptr<FriendsWithBenefitsClone>;
  std::unique_ptr<FriendsWithBenefitsClone> Clone() const {
    return std::make_unique<FriendsWithBenefitsClone>(value_);
  }

  int value_{0};
};

// Check that a friend declaration works to get copyable_unique_ptr access
// to the needed copy constructor or Clone() method.
GTEST_TEST(CopyableUniquePtrTest, FriendsWithBenefits) {
  copyable_unique_ptr<FriendsWithBenefitsCopy> fwb(
      new FriendsWithBenefitsCopy(10));
  EXPECT_EQ(fwb->value(), 10);

  copyable_unique_ptr<FriendsWithBenefitsCopy> fwb2(fwb);
  EXPECT_EQ(fwb2->value(), 10);

  copyable_unique_ptr<FriendsWithBenefitsClone> fwbc(
      new FriendsWithBenefitsClone(20));
  EXPECT_EQ(fwbc->value(), 20);

  copyable_unique_ptr<FriendsWithBenefitsClone> fwbc2(fwbc);
  EXPECT_EQ(fwbc2->value(), 20);
}

// A fully copyable class (has both copy constructor and Clone). Confirms that
// in the presence of both, the clone function is preferred.
struct FullyCopyable : Base {
  explicit FullyCopyable(int v, Origin org = Origin::CONSTRUCT)
      : Base(v, org) {}
  FullyCopyable(const FullyCopyable& b) : Base(b) {}
  unique_ptr<FullyCopyable> Clone() const {
    return make_unique<FullyCopyable>(value, Origin::CLONE);
  }
};

// Confirms that Clone is preferred when both exist.
GTEST_TEST(CopyableUniquePtrTest, FullyCopyableSuccess) {
  cup<FullyCopyable> ptr(new FullyCopyable(1));
  EXPECT_EQ(ptr->origin, Origin::CONSTRUCT);
  cup<FullyCopyable> copy(ptr);
  EXPECT_EQ(copy->origin, Origin::CLONE);
  EXPECT_EQ(copy->value, ptr->value);
  ++copy->value;
  EXPECT_NE(copy->value, ptr->value);
}

// A class with only a Clone method.
struct CloneOnly : Base {
  explicit CloneOnly(int v, Origin org = Origin::CONSTRUCT) : Base(v, org) {}
  unique_ptr<CloneOnly> Clone() const {
    return unique_ptr<CloneOnly>(DoClone());
  }
 protected:
  CloneOnly(const CloneOnly& other) : Base(other.value) {}
  [[nodiscard]] virtual CloneOnly* DoClone() const {
    return new CloneOnly(value, Origin::CLONE);
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

// A class that derives from a copyable class with no copy constructor. It
// provides a Clone method. This can be assigned to a cup<CloneOnly> *and*
// supports the cup<CloneOnlyChildWithClone> specialization. Copying an instance
// of cup<CloneOnly> which contains a reference to this will produce an instance
// of CloneOnlyChildWithClone.
struct CloneOnlyChildWithClone : CloneOnly {
  explicit CloneOnlyChildWithClone(int v, Origin org = Origin::CONSTRUCT)
      : CloneOnly(v, org) {}
  CloneOnlyChildWithClone(const CloneOnlyChildWithClone&) = delete;
  unique_ptr<CloneOnlyChildWithClone> Clone() const {
    return unique_ptr<CloneOnlyChildWithClone>(DoClone());
  }
 protected:
  [[nodiscard]] CloneOnlyChildWithClone* DoClone() const override {
    return new CloneOnlyChildWithClone(value, Origin::CLONE);
  }
};

// A class that derives from a copyable class with no copy constructor. It
// provides a copy constructor. This can be assigned to a cup<CloneOnly> *and*
// supports the cup<CloneOnlyWithCopy> specialization. It also overrides the
// protected DoClone method. Copying an instance of cup<CloneOnly> which
// contains a reference to this will produce a copy of
// CloneOnlyChildWithCopyVClone.
struct CloneOnlyChildWithCopyVClone : CloneOnly {
  explicit CloneOnlyChildWithCopyVClone(int v, Origin org = Origin::CONSTRUCT)
      : CloneOnly(v, org) {}
  CloneOnlyChildWithCopyVClone(const CloneOnlyChildWithCopyVClone&) = default;
 protected:
  [[nodiscard]] CloneOnlyChildWithCopyVClone* DoClone() const override {
    return new CloneOnlyChildWithCopyVClone(value, Origin::CLONE);
  }
};

// A class that derives from a copyable class with no copy constructor. It
// provides a copy constructor. This can be assigned to a cup<CloneOnly> *and*
// supports the cup<CloneOnlyWithCopy> specialization. This does *not* override
// the protected DoClone() method. Copying an instance of cup<CloneOnly> which
// contains a reference to this will produce a type-sliced copy of CloneOnly.
struct CloneOnlyChildWithCopy : CloneOnly {
  explicit CloneOnlyChildWithCopy(int v, Origin org = Origin::CONSTRUCT)
      : CloneOnly(v, org) {}
  CloneOnlyChildWithCopy(const CloneOnlyChildWithCopy&) = default;
};

// A class that derives from a copyable class with no copy constructor. It
// provides no copy functions. This can be assigned to a cup<CloneOnly> but does
// *not* support the cup<CloneOnlyChildUncopyable> specialization. Copying an
// instance of cup<CloneOnly> which contains a reference to this will produce a
// type-sliced copy of CloneOnly.
struct CloneOnlyChildUncopyable : CloneOnly {
  explicit CloneOnlyChildUncopyable(int v, Origin org = Origin::CONSTRUCT)
      : CloneOnly(v, org) {}
  CloneOnlyChildUncopyable(const CloneOnlyChildUncopyable&) = delete;
};

// A class that derives from a copyable class *with* a copy constructor. It
// provides its own copy constructor. This can be assigned to a cup<CopyChild>
// and supports the cup<CopyChild> specialization. Copying an instance of
// cup<FullyCopyable> which contains a reference to a CopyChild will produce a
// type-sliced copy of FullyCopyable.
struct CopyChild : public FullyCopyable {
  explicit CopyChild(int v) : FullyCopyable(v) {}
  CopyChild(const CopyChild& c) = default;
};

// Tests the copyability of derived class of a copyable class. In this case,
// the base class has *only* a Clone method and no copy constructor. Just
// because the base class is copyable does not imply the child is copyable.
GTEST_TEST(CopyableUniquePtrTest, PolymorphicCopyability) {
  // FYI is_cloneable and is_copy_constructible look only for public methods
  // which is a more stringent condition that copyable_unique_ptr enforces.
  // Below we'll verify that the presence of an appropriate public method *does*
  // enable us to use copyable_unique_ptr, by creating one of the appropriate
  // type and then marking it "unused" to avoid warnings. If this test compiles
  // then we succeeded.

  // Case 1) Child with *only* Clone method.
  EXPECT_TRUE(is_cloneable<CloneOnlyChildWithClone>::value);
  EXPECT_FALSE(std::is_copy_constructible_v<CloneOnlyChildWithClone>);
  copyable_unique_ptr<CloneOnlyChildWithClone> ptr_1;

  // Case 2) Child with *only* Copy method but virtual DoClone().
  EXPECT_FALSE(is_cloneable<CloneOnlyChildWithCopyVClone>::value);
  EXPECT_TRUE(std::is_copy_constructible_v<CloneOnlyChildWithCopyVClone>);
  copyable_unique_ptr<CloneOnlyChildWithCopyVClone> ptr_2;

  // Case 3) Child with *only* Copy method.
  EXPECT_FALSE(is_cloneable<CloneOnlyChildWithCopy>::value);
  EXPECT_TRUE(std::is_copy_constructible_v<CloneOnlyChildWithCopy>);
  copyable_unique_ptr<CloneOnlyChildWithCopy> ptr_3;

  // Case 4) Child with no copy and no clone.
  EXPECT_FALSE(is_cloneable<CloneOnlyChildUncopyable>::value);
  EXPECT_FALSE(std::is_copy_constructible_v<CloneOnlyChildUncopyable>);
  // Can't make a cloneable_unique_ptr<CloneOnlyChildUncopyable>.

  // Case 5) Child with copy, derived from base with copy.
  EXPECT_FALSE(is_cloneable<CopyChild>::value);
  EXPECT_TRUE(std::is_copy_constructible_v<CopyChild>);
  copyable_unique_ptr<CopyChild> ptr_4;

  unused(ptr_1, ptr_2, ptr_3, ptr_4);
}

// Utility test for the CopyTypeSlicing test. It is templated on the sub-class
// of CloneOnly. Various implementations in the derived class can lead to
// copies that slice the type back to CloneOnly.
template <typename T>
void TestPolymorphicCopy(bool copy_success) {
  static_assert(std::is_convertible_v<T*, CloneOnly*>,
                "This utility method can only be used with classes derived "
                "from CloneOnly.");
  cup<CloneOnly> src(new T(1));
  cup<CloneOnly> tgt;
  tgt = src;    // Triggers a copy
  EXPECT_NE(tgt.get(), nullptr);    // Confirm actual object assigned.
  EXPECT_NE(tgt.get(), src.get());  // Confirm different objects.
  if (copy_success) {
    EXPECT_TRUE(is_dynamic_castable<T>(tgt.get()));
  } else {
    EXPECT_TRUE((std::is_same_v<const CloneOnly*, decltype(tgt.get())>));
    EXPECT_FALSE(is_dynamic_castable<T>(tgt.get()));
  }
}

// Tests the copy functionality based on polymorphism. Given a
// copyable_unique_ptr on a base class, various concrete derived instances are
// pushed into the pointer and copied. Some derived classes will be type
// sliced and some will have their type preserved. This confirms the behavior.
GTEST_TEST(CopyableUniquePtrTest, CopyTypeSlicing) {
  // Case 1) Child with *only* Clone method.
  TestPolymorphicCopy<CloneOnlyChildWithClone>(true);
  // Case 2) Child with *only* Copy method but protected DoClone() override.
  TestPolymorphicCopy<CloneOnlyChildWithCopyVClone>(true);
  // Case 3) Child with *only* Copy method.
  TestPolymorphicCopy<CloneOnlyChildWithCopy>(false);
  // Case 4) Child with no copy and no clone.
  TestPolymorphicCopy<CloneOnlyChildUncopyable>(false);
  // Case 5) Child with copy, derived from base with copy.
  cup<FullyCopyable> src(new CopyChild(1));
  cup<FullyCopyable> tgt;
  tgt = src;    // Triggers a copy
  EXPECT_NE(tgt.get(), nullptr);    // Confirm actual object assigned.
  EXPECT_NE(tgt.get(), src.get());  // Confirm different objects.
  EXPECT_TRUE((std::is_same_v<const FullyCopyable*, decltype(tgt.get())>));
  EXPECT_FALSE(is_dynamic_castable<CopyChild>(tgt.get()));
}

// This tests the structure of a class and confirms that the "is_copyable" value
// conforms to the stated properties. A class is copyable if it is copy
// constructible *or* cloneable.
GTEST_TEST(CopyableUniquePtrTest, CopyableAsExpected) {
  // Examine the 2 x 2 matrix of copy constructible X cloneable with copyable
  // indicated in parentheses and case numbers in the corners:
  //
  //            copy constructible
  //              T           F
  //         ___________________________
  //   c    |1)             |2)         |
  //   l    |               |           |
  //   o  T | FullyCopyable | CloneOnly |
  //   n    |      (T)      |    (T)    |
  //   e    |_______________|___________|
  //   a    |3)             |4)         |
  //   b    |               |           |
  //   l  F |   CopyOnly    |    Base   |
  //   e    |     (T)       |     (F)   |
  //        |_______________|___________|
  //

  // Case X) Cloneable || Constructible --> Copyable

  // Case 1) True || True --> True
  EXPECT_TRUE(is_cloneable<FullyCopyable>::value);
  EXPECT_TRUE(std::is_copy_constructible_v<FullyCopyable>);

  // Case 2) True || False --> True
  EXPECT_TRUE(is_cloneable<CloneOnly>::value);
  EXPECT_FALSE(std::is_copy_constructible_v<CloneOnly>);

  // Case 3) False || True --> True
  EXPECT_FALSE(is_cloneable<CopyOnly>::value);
  EXPECT_TRUE(std::is_copy_constructible_v<CopyOnly>);

  // Case 4) False || False --> False
  EXPECT_FALSE(is_cloneable<Base>::value);
  EXPECT_FALSE(std::is_copy_constructible_v<Base>);
}

// ------------------------ Constructor Tests ------------------------------
// These tests cover construction functionality.

// Tests constructor methods that create an empty pointer. This implicitly
// relies on the empty() method.
GTEST_TEST(CopyableUniquePtrTest, ConstructEmptyPtr) {
  // Default constructor.
  cup<CopyOnly> ptr;
  EXPECT_TRUE(ptr.empty());

  // Explicitly null.
  cup<CopyOnly> ptr2(nullptr);
  EXPECT_TRUE(ptr2.empty());
}

// Test constructor methods that construct a valid copyable_unique_ptr from
// compatible non-null pointers. These constructors do *not* invoke any copying.
GTEST_TEST(CopyableUniquePtrTest, ConstructOnPtrNoCopy) {
  CloneOnly* base_ptr;
  // Case 1: cup<Base> with Base*
  cup<CloneOnly> ptr(base_ptr = new CloneOnly(1));
  EXPECT_EQ(ptr.get(), base_ptr);

  // Case 2: cup<Base> with Derived*
  CloneOnlyChildWithClone* co_ptr;
  cup<CloneOnly> ptr2(co_ptr = new CloneOnlyChildWithClone(2));
  // Shows that type is preserved.
  ASSERT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(ptr2.get()));
  EXPECT_EQ(ptr2.get(), co_ptr);
}

// Test copy constructor on copyable_unique_ptr. Copying from three different
// types:
//    copyable_unique_ptr<Base> (containing a Base*)
//    copyable_unique_ptr<Base> (containing a Derived*)
//    copyable_unique_ptr<Derived> (containing a Derived*)
GTEST_TEST(CopyableUniquePtrTest, CopyConstructFromCopyable) {
  CloneOnly* base_ptr;
  cup<CloneOnly> u_ptr(base_ptr = new CloneOnly(1));
  EXPECT_EQ(u_ptr.get(), base_ptr);
  // Copy constructor on copyable_unique-ptr of same specialized class.
  cup<CloneOnly> cup_ptr(u_ptr);
  EXPECT_EQ(u_ptr.get(), base_ptr);
  EXPECT_NE(cup_ptr.get(), base_ptr);
  EXPECT_NE(cup_ptr.get(), nullptr);
  EXPECT_EQ(cup_ptr->value, u_ptr->value);

  CloneOnlyChildWithClone* co_ptr;
  cup<CloneOnly> u_ptr2(co_ptr = new CloneOnlyChildWithClone(2));
  EXPECT_EQ(u_ptr2.get(), co_ptr);
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(u_ptr2.get()));
  // Copy constructor on copyable_unique-ptr of same specialized class, but
  // contains derived class.
  cup<CloneOnly> cup_ptr2(u_ptr2);
  EXPECT_EQ(u_ptr2.get(), co_ptr);
  EXPECT_NE(cup_ptr2.get(), co_ptr);
  EXPECT_NE(cup_ptr2.get(), nullptr);
  EXPECT_EQ(cup_ptr2->value, u_ptr2->value);
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(cup_ptr2.get()));

  // Copy constructor on copyable_unique-ptr of derived specialized class.
  CloneOnlyChildWithClone* co_ptr3;
  cup<CloneOnlyChildWithClone> u_ptr3(co_ptr3 = new CloneOnlyChildWithClone(3));
  EXPECT_EQ(u_ptr3.get(), co_ptr3);
  cup<CloneOnly> cup_ptr3(u_ptr3);
  EXPECT_EQ(u_ptr3.get(), co_ptr3);
  EXPECT_NE(cup_ptr3.get(), co_ptr3);
  EXPECT_NE(cup_ptr3.get(), nullptr);
  EXPECT_EQ(cup_ptr3->value, u_ptr3->value);
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(cup_ptr3.get()));
}

// Test copy constructor on unique_ptr. Copying from three different
// types:
//    copyable_unique_ptr<Base> (containing a Base*)
//    copyable_unique_ptr<Base> (containing a Derived*)
//    copyable_unique_ptr<Derived> (containing a Derived*)
GTEST_TEST(CopyableUniquePtrTest, CopyConstructFromUniquePtr) {
  CloneOnly* base_ptr;
  unique_ptr<CloneOnly> u_ptr(base_ptr = new CloneOnly(1));
  EXPECT_EQ(u_ptr.get(), base_ptr);
  // Copy constructor on copyable_unique-ptr of same specialized class.
  cup<CloneOnly> cup_ptr(u_ptr);
  EXPECT_EQ(u_ptr.get(), base_ptr);
  EXPECT_NE(cup_ptr.get(), base_ptr);
  EXPECT_NE(cup_ptr.get(), nullptr);
  EXPECT_EQ(cup_ptr->value, u_ptr->value);

  CloneOnlyChildWithClone* co_ptr;
  unique_ptr<CloneOnly> u_ptr2(co_ptr = new CloneOnlyChildWithClone(2));
  EXPECT_EQ(u_ptr2.get(), co_ptr);
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(u_ptr2.get()));
  // Copy constructor on copyable_unique-ptr of same specialized class, but
  // contains derived class.
  cup<CloneOnly> cup_ptr2(u_ptr2);
  EXPECT_EQ(u_ptr2.get(), co_ptr);
  EXPECT_NE(cup_ptr2.get(), co_ptr);
  EXPECT_NE(cup_ptr2.get(), nullptr);
  EXPECT_EQ(cup_ptr2->value, u_ptr2->value);
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(cup_ptr2.get()));

  // Copy constructor on copyable_unique-ptr of derived specialized class.
  CloneOnlyChildWithClone* co_ptr3;
  unique_ptr<CloneOnlyChildWithClone> u_ptr3(
      co_ptr3 = new CloneOnlyChildWithClone(3));
  EXPECT_EQ(u_ptr3.get(), co_ptr3);
  cup<CloneOnly> cup_ptr3(u_ptr3);
  EXPECT_EQ(u_ptr3.get(), co_ptr3);
  EXPECT_NE(cup_ptr3.get(), co_ptr3);
  EXPECT_NE(cup_ptr3.get(), nullptr);
  EXPECT_EQ(cup_ptr3->value, u_ptr3->value);
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(cup_ptr3.get()));
}

// Test move constructor on copyable_unique_ptr. Copying from three different
// types:
//    copyable_unique_ptr<Base> (containing a Base*)
//    copyable_unique_ptr<Base> (containing a Derived*)
//    copyable_unique_ptr<Derived> (containing a Derived*)
GTEST_TEST(CopyableUniquePtrTest, MoveConstructFromCopyable) {
  CloneOnly* base_ptr;
  cup<CloneOnly> u_ptr(base_ptr = new CloneOnly(1));
  EXPECT_EQ(u_ptr.get(), base_ptr);
  // Move constructor on copyable_unique-ptr of same specialized class.
  cup<CloneOnly> cup_ptr(move(u_ptr));
  EXPECT_EQ(u_ptr.get(), nullptr);
  EXPECT_EQ(cup_ptr.get(), base_ptr);

  CloneOnlyChildWithClone* co_ptr;
  cup<CloneOnly> u_ptr2(co_ptr = new CloneOnlyChildWithClone(2));
  EXPECT_EQ(u_ptr2.get(), co_ptr);
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(u_ptr2.get()));
  // Copy constructor on copyable_unique-ptr of same specialized class, but
  // contains derived class.
  cup<CloneOnly> cup_ptr2(move(u_ptr2));
  EXPECT_EQ(u_ptr2.get(), nullptr);
  EXPECT_EQ(cup_ptr2.get(), co_ptr);
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(cup_ptr2.get()));

  // Copy constructor on copyable_unique-ptr of derived specialized class.
  CloneOnlyChildWithClone* co_ptr3;
  cup<CloneOnlyChildWithClone> u_ptr3(co_ptr3 = new CloneOnlyChildWithClone(3));
  EXPECT_EQ(u_ptr3.get(), co_ptr3);
  cup<CloneOnly> cup_ptr3(move(u_ptr3));
  EXPECT_EQ(u_ptr3.get(), nullptr);
  EXPECT_EQ(cup_ptr3.get(), co_ptr3);
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(cup_ptr3.get()));
}

// Test move constructor on unique_ptr. Copying from three different
// types:
//    copyable_unique_ptr<Base> (containing a Base*)
//    copyable_unique_ptr<Base> (containing a Derived*)
//    copyable_unique_ptr<Derived> (containing a Derived*)
GTEST_TEST(CopyableUniquePtrTest, MoveConstructFromUnique) {
  CloneOnly* base_ptr;
  unique_ptr<CloneOnly> u_ptr(base_ptr = new CloneOnly(1));
  EXPECT_EQ(u_ptr.get(), base_ptr);
  // Move constructor on copyable_unique-ptr of same specialized class.
  cup<CloneOnly> cup_ptr(move(u_ptr));
  EXPECT_EQ(u_ptr.get(), nullptr);
  EXPECT_EQ(cup_ptr.get(), base_ptr);

  CloneOnlyChildWithClone* co_ptr;
  unique_ptr<CloneOnly> u_ptr2(co_ptr = new CloneOnlyChildWithClone(2));
  EXPECT_EQ(u_ptr2.get(), co_ptr);
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(u_ptr2.get()));
  // Copy constructor on copyable_unique-ptr of same specialized class, but
  // contains derived class.
  cup<CloneOnly> cup_ptr2(move(u_ptr2));
  EXPECT_EQ(u_ptr2.get(), nullptr);
  EXPECT_EQ(cup_ptr2.get(), co_ptr);
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(cup_ptr2.get()));

  // Copy constructor on copyable_unique-ptr of derived specialized class.
  CloneOnlyChildWithClone* co_ptr3;
  unique_ptr<CloneOnlyChildWithClone> u_ptr3(
      co_ptr3 = new CloneOnlyChildWithClone(3));
  EXPECT_EQ(u_ptr3.get(), co_ptr3);
  cup<CloneOnly> cup_ptr3(move(u_ptr3));
  EXPECT_EQ(u_ptr3.get(), nullptr);
  EXPECT_EQ(cup_ptr3.get(), co_ptr3);
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(cup_ptr3.get()));
}

// Test the construction from a sample model value. We don't test all varieties
// of cloneable types because we know that the model construction uses the same
// underlying mechanism as has been tested by all the other constructors.
GTEST_TEST(CopyableUniquePtrTest, ConstructByValue) {
  CloneOnly clone_only(17);
  cup<CloneOnly> clone_ptr(clone_only);
  EXPECT_NE(&clone_only, clone_ptr.get());
  EXPECT_EQ(clone_only.value, clone_ptr->value);

  CopyOnly copy_only(2);
  cup<CopyOnly> copy_ptr(copy_only);
  EXPECT_NE(&copy_only, copy_ptr.get());
  EXPECT_EQ(copy_only.value, copy_ptr->value);
}

// ------------------------ Destructor Tests ------------------------------
// These tests the various methods that cause the object to be destroyed.

// This class allows me to track destruction of instances in copyable_unique_ptr
// instances. To use it, set the static member dtor_called to false, and then
// perform an operation. If an instance has been destroyed, dtor_called will be
// true, otherwise, no destructor was called.
struct DestructorTracker : public CloneOnly {
  explicit DestructorTracker(int v, Origin org = Origin::CONSTRUCT)
      : CloneOnly(v, org) {}
  virtual ~DestructorTracker() { dtor_called = true; }
  DestructorTracker(const DestructorTracker&) = delete;
  unique_ptr<DestructorTracker> Clone() const {
    return make_unique<DestructorTracker>(value, Origin::CLONE);
  }
  static bool dtor_called;
};
bool DestructorTracker::dtor_called = false;

// Tests the destruction of the referenced object when the pointer is destroyed.
// In this case, destruction is triggered by leaving the scope of the
// copyable_unique_ptr.
GTEST_TEST(CopyableUniquePtrTest, DestroyOnScopeExit) {
  // Empty copyable_unique_ptr<T> does not invoke T's destructor.
  {
    cup<DestructorTracker> empty;
    DestructorTracker::dtor_called = false;
  }
  EXPECT_FALSE(DestructorTracker::dtor_called);

  // Non-empty pointer invokes destructor.
  DestructorTracker* raw_ptr = new DestructorTracker(1);
  {
    cup<DestructorTracker> ptr(raw_ptr);
    EXPECT_EQ(ptr.get(), raw_ptr);
    DestructorTracker::dtor_called = false;
  }
  EXPECT_TRUE(DestructorTracker::dtor_called);
}

// This tests the various incarnations of reset.
GTEST_TEST(CopyableUniquePtrTest, Reset) {
  cup<DestructorTracker> ptr;
  EXPECT_TRUE(ptr.empty());  // Confirm initial condition.

  // Case 1: Resetting an empty pointer invokes no constructor.
  DestructorTracker* raw_ptr = new DestructorTracker(1);
  DestructorTracker::dtor_called = false;
  ptr.reset(raw_ptr);
  EXPECT_EQ(ptr.get(), raw_ptr);
  // Previously empty pointer does *not* invoke destructor.
  EXPECT_FALSE(DestructorTracker::dtor_called);

  // Case 2: Previous non-null contents *are* destroyed.
  raw_ptr = new DestructorTracker(2);
  EXPECT_NE(ptr.get(), raw_ptr);
  DestructorTracker::dtor_called = false;
  ptr.reset(raw_ptr);
  EXPECT_EQ(ptr.get(), raw_ptr);
  // Previously non-empty pointer *does* invoke destructor.
  EXPECT_TRUE(DestructorTracker::dtor_called);

  // Case 3: Previous non-null contents replace by nullptr.
  DestructorTracker::dtor_called = false;
  ptr.reset();
  EXPECT_EQ(ptr.get(), nullptr);
  // Previously non-empty pointer *does* invoke destructor.
  EXPECT_TRUE(DestructorTracker::dtor_called);
}

// ------------------------ Assignment Tests ------------------------------
// These tests cover assignment operator.

// Tests the assignment of a raw pointer to a copyable_unique_ptr. The
// copyable_unique_ptr should take ownership. It also confirms that the
// previous object is destroyed. Assignment of the following patterns:
//
// | Target Type | Source   | Target State | Dtor? |
// |-------------+----------+--------------+-------|
// | Base        | nullptr  | nullptr      | N     |
// | Base        | Base*    | nullptr      | Y     |
// | Base        | Base*    | Base*        | Y     |
// | Base        | Derived* | nullptr      | N     |
GTEST_TEST(CopyableUniquePtrTest, AssignPointer) {
  cup<DestructorTracker> ptr;

  // Do not re-order these tests. Each test implicitly relies on the verified
  // state at the conclusion of the previous case.

  // Case 1: Assign nullptr to empty cup.
  EXPECT_EQ(ptr.get(), nullptr);
  DestructorTracker::dtor_called = false;
  ptr = nullptr;
  EXPECT_FALSE(DestructorTracker::dtor_called);
  EXPECT_EQ(ptr.get(), nullptr);

  // Case 2: Assign pointer of specialized type to empty cup.
  DestructorTracker::dtor_called = false;
  DestructorTracker* raw = new DestructorTracker(421);
  ptr = raw;
  EXPECT_FALSE(DestructorTracker::dtor_called);
  EXPECT_EQ(ptr.get(), raw);

  // Case 3: Assign pointer of specialized type to non-empty cup.
  DestructorTracker* raw2 = new DestructorTracker(124);
  DestructorTracker::dtor_called = false;
  ptr = raw2;
  EXPECT_TRUE(DestructorTracker::dtor_called);
  EXPECT_EQ(ptr.get(), raw2);

  // Case 4: Assign pointer of Derived type to empty cup<Base>
  cup<CloneOnly> co_ptr;
  CloneOnlyChildWithClone* derived_raw = new CloneOnlyChildWithClone(13);
  co_ptr = derived_raw;
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(co_ptr.get()));
  EXPECT_EQ(co_ptr.get(), derived_raw);
}

// Tests the assignment of a const reference object to a copyable_unique_ptr.
// This should *always* create a copy. It also confirms that the previous object
// is destroyed. Assignment of the following patterns:
//
// | Target Type | Source   | Target State | Dtor? |
// |-------------+----------+--------------+-------|
// | Base        | Base&    | nullptr      | Y     |
// | Base        | Base&    | Base*        | Y     |
// | Base        | Derived& | nullptr      | N     |
GTEST_TEST(CopyableUniquePtrTest, AssignConstReference) {
  DestructorTracker ref(421);
  cup<DestructorTracker> ptr;

  // Do not re-order these tests. Each test implicitly relies on the verified
  // state at the conclusion of the previous case.

  // Case 1: Assign reference of specialized type to empty cup.
  EXPECT_EQ(ptr.get(), nullptr);
  DestructorTracker::dtor_called = false;
  ptr = ref;
  EXPECT_FALSE(DestructorTracker::dtor_called);
  EXPECT_NE(ptr.get(), &ref);
  EXPECT_EQ(ptr->value, ref.value);

  // Case 2: Assign reference of specialized type to non-empty cup.
  DestructorTracker ref2(124);
  DestructorTracker::dtor_called = false;
  ptr = ref2;
  EXPECT_TRUE(DestructorTracker::dtor_called);
  EXPECT_NE(ptr.get(), &ref);
  EXPECT_NE(ptr.get(), &ref2);
  EXPECT_EQ(ptr->value, ref2.value);

  // Case 3: Assign reference of Derived type to empty cup<Base>
  cup<CloneOnly> co_ptr;
  CloneOnlyChildWithClone derived_ref(132);
  co_ptr = derived_ref;
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(co_ptr.get()));
  EXPECT_NE(co_ptr.get(), &derived_ref);
  EXPECT_EQ(co_ptr->value, derived_ref.value);
}

// Tests the copy assignment of a copyable_unique_ptr to a copyable_unique_ptr.
// This should *always* create a copy. It also confirms that the previous object
// is destroyed. Assignment of the following patterns:
//
// | Target Type | Source    | Target State | Dtor? |
// |-------------+-----------+--------------+-------|
// | Base        | empty     | nullptr      | N     |
// | Base        | <Base>    | nullptr      | Y     |
// | Base        | <Base>    | Base*        | Y     |
// | Base        | <Derived> | nullptr      | N     |
GTEST_TEST(CopyableUniquePtrTest, CopyAssignFromCopyableUniquePtr) {
  DestructorTracker* raw = new DestructorTracker(421);
  cup<DestructorTracker> src(raw);
  cup<DestructorTracker> empty;
  cup<DestructorTracker> tgt;

  // Do not re-order these tests. Each test implicitly relies on the verified
  // state at the conclusion of the previous case.

  // Case 1: Assign empty cup to empty cup.
  EXPECT_EQ(tgt.get(), nullptr);
  DestructorTracker::dtor_called = false;
  tgt = empty;
  EXPECT_FALSE(DestructorTracker::dtor_called);
  EXPECT_EQ(tgt.get(), nullptr);
  EXPECT_EQ(empty.get(), nullptr);

  // Case 2: Assign non-empty cup<Base> to empty cup<Base>.
  DestructorTracker::dtor_called = false;
  tgt = src;
  EXPECT_FALSE(DestructorTracker::dtor_called);
  EXPECT_NE(tgt.get(), src.get());    // Not the same pointer as src.
  EXPECT_EQ(src.get(), raw);          // Src has original value.
  EXPECT_EQ(tgt->value, src->value);  // Src and tgt are copies.

  // Case 3: Assign non-empty cup<Base> type to non-empty cup<Base>.
  DestructorTracker* raw2;
  cup<DestructorTracker> src2(raw2 = new DestructorTracker(124));
  DestructorTracker::dtor_called = false;
  tgt = src2;
  EXPECT_TRUE(DestructorTracker::dtor_called);
  EXPECT_NE(tgt.get(), src2.get());     // Not the same pointer as src.
  EXPECT_EQ(src2.get(), raw2);          // Src has original value.
  EXPECT_EQ(tgt->value, src2->value);   // Src and tgt are copies.

  // Case 4: Assign non-empty cup<Derived> to empty cup<Base>
  cup<CloneOnly> base_tgt;
  CloneOnlyChildWithClone* derived_raw;
  cup<CloneOnlyChildWithClone> derived_src(derived_raw =
                                               new CloneOnlyChildWithClone(13));
  base_tgt = derived_src;
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(base_tgt.get()));
  EXPECT_NE(base_tgt.get(), derived_src.get());
  EXPECT_EQ(base_tgt->value, derived_src->value);
}

// Tests the copy assignment of a standard unique_ptr to a copyable_unique_ptr.
// This should *always* create a copy. It also confirms that the previous object
// is destroyed. Assignment of the following patterns:
//
// | Target Type | Source    | Target State | Dtor? |
// |-------------+-----------+--------------+-------|
// | Base        | empty     | nullptr      | N     |
// | Base        | <Base>    | nullptr      | Y     |
// | Base        | <Base>    | Base*        | Y     |
// | Base        | <Derived> | nullptr      | N     |
GTEST_TEST(CopyableUniquePtrTest, CopyAssignFromUniquePtr) {
  DestructorTracker* raw = new DestructorTracker(421);
  unique_ptr<DestructorTracker> src(raw);
  unique_ptr<DestructorTracker> empty;
  cup<DestructorTracker> tgt;

  // Do not re-order these tests. Each test implicitly relies on the verified
  // state at the conclusion of the previous case.

  // Case 1: Assign empty unique_ptr to empty cup.
  EXPECT_EQ(tgt.get(), nullptr);
  DestructorTracker::dtor_called = false;
  tgt = empty;
  EXPECT_FALSE(DestructorTracker::dtor_called);
  EXPECT_EQ(tgt.get(), nullptr);
  EXPECT_EQ(empty.get(), nullptr);

  // Case 2: Assign non-empty unique_ptr<Base> to empty cup<Base>.
  DestructorTracker::dtor_called = false;
  tgt = src;
  EXPECT_FALSE(DestructorTracker::dtor_called);
  EXPECT_NE(tgt.get(), src.get());    // Not the same pointer as src.
  EXPECT_EQ(src.get(), raw);          // Src has original value.
  EXPECT_EQ(tgt->value, src->value);  // Src and tgt are copies.

  // Case 3: Assign non-empty unique_ptr<Base> type to non-empty cup<Base>.
  DestructorTracker* raw2;
  cup<DestructorTracker> src2(raw2 = new DestructorTracker(124));
  DestructorTracker::dtor_called = false;
  tgt = src2;
  EXPECT_TRUE(DestructorTracker::dtor_called);
  EXPECT_NE(tgt.get(), src2.get());     // Not the same pointer as src.
  EXPECT_EQ(src2.get(), raw2);          // Src has original value.
  EXPECT_EQ(tgt->value, src2->value);   // Src and tgt are copies.

  // Case 4: Assign non-empty unique_ptr<Derived> to empty cup<Base>
  cup<CloneOnly> base_tgt;
  CloneOnlyChildWithClone* derived_raw;
  unique_ptr<CloneOnlyChildWithClone> derived_src(
      derived_raw = new CloneOnlyChildWithClone(13));
  base_tgt = derived_src;
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(base_tgt.get()));
  EXPECT_NE(base_tgt.get(), derived_src.get());
  EXPECT_EQ(base_tgt->value, derived_src->value);
}

// Tests the move assignment of a copyable_unique_ptr to a copyable_unique_ptr.
// This should reassign ownership changing the target and source. It also
// confirms that the previous object is destroyed. Assignment of the following
// patterns:
//
// | Target Type | Source    | Target State | Dtor? |
// |-------------+-----------+--------------+-------|
// | Base        | empty     | nullptr      | N     |
// | Base        | <Base>    | nullptr      | Y     |
// | Base        | <Base>    | Base*        | Y     |
// | Base        | <Derived> | nullptr      | N     |
GTEST_TEST(CopyableUniquePtrTest, MoveAssignFromCopyableUniquePtr) {
  DestructorTracker* raw = new DestructorTracker(421);
  cup<DestructorTracker> src(raw);
  cup<DestructorTracker> empty;
  cup<DestructorTracker> tgt;

  // Do not re-order these tests. Each test implicitly relies on the verified
  // state at the conclusion of the previous case.

  // Case 1: Assign empty unique_ptr to empty cup.
  EXPECT_EQ(tgt.get(), nullptr);
  DestructorTracker::dtor_called = false;
  tgt = move(empty);
  EXPECT_FALSE(DestructorTracker::dtor_called);
  EXPECT_EQ(tgt.get(), nullptr);
  EXPECT_EQ(empty.get(), nullptr);

  // Case 2: Assign non-empty unique_ptr<Base> to empty cup<Base>.
  DestructorTracker::dtor_called = false;
  tgt = move(src);
  EXPECT_FALSE(DestructorTracker::dtor_called);
  EXPECT_EQ(tgt.get(), raw);      // Tgt has taken ownership of raw.
  EXPECT_EQ(src.get(), nullptr);  // Src has been cleared.

  // Case 3: Assign non-empty unique_ptr<Base> type to non-empty cup<Base>.
  DestructorTracker* raw2;
  cup<DestructorTracker> src2(raw2 = new DestructorTracker(124));
  DestructorTracker::dtor_called = false;
  tgt = move(src2);
  EXPECT_TRUE(DestructorTracker::dtor_called);
  EXPECT_EQ(tgt.get(), raw2);       // Tgt has taken ownership of raw.
  EXPECT_EQ(src2.get(), nullptr);   // Src has been cleared.

  // Case 4: Assign non-empty unique_ptr<Derived> to empty cup<Base>
  cup<CloneOnly> base_tgt;
  CloneOnlyChildWithClone* derived_raw;
  cup<CloneOnlyChildWithClone> derived_src(derived_raw =
                                               new CloneOnlyChildWithClone(13));
  base_tgt = move(derived_src);
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(base_tgt.get()));
  EXPECT_EQ(base_tgt.get(), derived_raw);   // Tgt has taken ownership of raw.
  EXPECT_EQ(derived_src.get(), nullptr);    // Src has been cleared.
}

// Tests the move assignment of a standard unique_ptr to a copyable_unique_ptr.
// This should reassign ownership changing the target and source. It also
// confirms that the previous object is destroyed. Assignment of the following
// patterns:
//
// | Target Type | Source    | Target State | Dtor? |
// |-------------+-----------+--------------+-------|
// | Base        | empty     | nullptr      | N     |
// | Base        | <Base>    | nullptr      | Y     |
// | Base        | <Base>    | Base*        | Y     |
// | Base        | <Derived> | nullptr      | N     |
GTEST_TEST(CopyableUniquePtrTest, MoveAssignFromUniquePtr) {
  DestructorTracker* raw = new DestructorTracker(421);
  unique_ptr<DestructorTracker> src(raw);
  unique_ptr<DestructorTracker> empty;
  cup<DestructorTracker> tgt;

  // Do not re-order these tests. Each test implicitly relies on the verified
  // state at the conclusion of the previous case.

  // Case 1: Assign empty unique_ptr to empty cup.
  EXPECT_EQ(tgt.get(), nullptr);
  DestructorTracker::dtor_called = false;
  tgt = move(empty);
  EXPECT_FALSE(DestructorTracker::dtor_called);
  EXPECT_EQ(tgt.get(), nullptr);
  EXPECT_EQ(empty.get(), nullptr);

  // Case 2: Assign non-empty unique_ptr<Base> to empty cup<Base>.
  DestructorTracker::dtor_called = false;
  tgt = move(src);
  EXPECT_FALSE(DestructorTracker::dtor_called);
  EXPECT_EQ(tgt.get(), raw);      // Tgt has taken ownership of raw.
  EXPECT_EQ(src.get(), nullptr);  // Src has been cleared.

  // Case 3: Assign non-empty unique_ptr<Base> type to non-empty cup<Base>.
  DestructorTracker* raw2;
  cup<DestructorTracker> src2(raw2 = new DestructorTracker(124));
  DestructorTracker::dtor_called = false;
  tgt = move(src2);
  EXPECT_TRUE(DestructorTracker::dtor_called);
  EXPECT_EQ(tgt.get(), raw2);       // Tgt has taken ownership of raw.
  EXPECT_EQ(src2.get(), nullptr);   // Src has been cleared.

  // Case 4: Assign non-empty unique_ptr<Derived> to empty cup<Base>
  cup<CloneOnly> base_tgt;
  CloneOnlyChildWithClone* derived_raw;
  unique_ptr<CloneOnlyChildWithClone> derived_src(
      derived_raw = new CloneOnlyChildWithClone(13));
  base_tgt = move(derived_src);
  EXPECT_TRUE(is_dynamic_castable<CloneOnlyChildWithClone>(base_tgt.get()));
  EXPECT_EQ(base_tgt.get(), derived_raw);   // Tgt has taken ownership of raw.
  EXPECT_EQ(derived_src.get(), nullptr);    // Src has been cleared.
}

// ------------------------ Observer Tests ------------------------------
// These tests cover the introspective functions: determining if the pointer is
// empty, acquiring access to the raw pointer, and references. The empty()
// method is implicitly tested multiple times in other methods.


// Tests the semantics that get() and get_mutable() return const and non-const
// pointers respectively.
GTEST_TEST(CopyableUniquePtrTest, PointerAccessConstSemantics) {
  CloneOnly* raw;
  cup<CloneOnly> ptr(raw = new CloneOnly(1));

  // Simply test that they return the right pointer *value*.
  EXPECT_EQ(ptr.get(), raw);
  EXPECT_EQ(ptr.get_mutable(), raw);

  // The extra parentheses prevent the macro from getting confused.
  // get is const pointer
  EXPECT_TRUE((std::is_same_v<const CloneOnly*, decltype(ptr.get())>));
  // NOTE: is_assignable uses declval<T> to create the lhs. declval<T> creates
  // an r-value reference (which will *always* fail assignability tests. By
  // taking an l-value reference of the object, the resulting type becomes
  // l-value.
  EXPECT_FALSE((std::is_assignable_v<CloneOnly*&, decltype(ptr.get())>));

  // get_mutable is non-const
  EXPECT_TRUE((std::is_same_v<CloneOnly*, decltype(ptr.get_mutable())>));
  EXPECT_TRUE((std::is_assignable_v<CloneOnly*&, decltype(ptr.get_mutable())>));

  // Constness of dereferenced result depends on the constness of the smart
  // pointer.
  EXPECT_TRUE((std::is_same_v<CloneOnly&, decltype(*ptr)>));
  const cup<CloneOnly>& const_ptr_ref = ptr;
  EXPECT_TRUE((std::is_same_v<const CloneOnly&, decltype(*const_ptr_ref)>));

  // Constness of T cannot be undone by non-const pointer container.
  cup<const CloneOnly> ptr_to_const(new CloneOnly(2));
  EXPECT_TRUE(
      (std::is_same_v<const CloneOnly*, decltype(ptr_to_const.get_mutable())>));
  EXPECT_TRUE((std::is_same_v<const CloneOnly&, decltype(*ptr_to_const)>));
}

// ------------------------ Core unique_ptr Tests ------------------------------
// This tests the functionality that should be directly inherited from the
// unique_ptr class, preserving the idea that, except for copying semantics,
// the copyable_unique_ptr *is* a unique_ptr.

// This tests that a cup<const T> offers no mutable access to the underlying
// // type (via pointer or reference).
GTEST_TEST(CopyableUniquePtrTest, ConstSpecializationHasNoMutableAccess) {
  cup<const CloneOnly> ptr(new CloneOnly(2));
  // Being the same as a 'const' type, precludes the possibility of being the
  // "same" as a non-const type.
  EXPECT_TRUE((std::is_same_v<const CloneOnly*, decltype(ptr.get())>));
}

// This tests the implicit conversion of the pointer to a boolean. It does *not*
// confirm that it is only available in "contextual conversions". See:
// http://en.cppreference.com/w/cpp/language/implicit_conversion#Contextual_conversions
GTEST_TEST(CopyableUniquePtrTest, ConverstionToBool) {
  copyable_unique_ptr<CloneOnly> ptr;
  EXPECT_FALSE(ptr);
  ptr.reset(new CloneOnly(1));
  EXPECT_TRUE(ptr);
}

// Tests the two forms of swap (member method and external method).
GTEST_TEST(CopyableUniquePtrTest, SwapTest) {
  DestructorTracker* raw1;
  DestructorTracker* raw2;
  cup<DestructorTracker> ptr1(raw1 = new DestructorTracker(1));
  cup<DestructorTracker> ptr2(raw2 = new DestructorTracker(2));
  EXPECT_EQ(ptr1.get(), raw1);
  EXPECT_EQ(ptr2.get(), raw2);
  EXPECT_NE(ptr1.get(), ptr2.get());

  // Don't reorder these tests. Results depend on this execution order.

  // The swap method.
  DestructorTracker::dtor_called = false;
  ptr1.swap(ptr2);
  EXPECT_FALSE(DestructorTracker::dtor_called);  // Nothing got destroyed.
  EXPECT_EQ(ptr1.get(), raw2);
  EXPECT_EQ(ptr2.get(), raw1);

  // The swap function.
  DestructorTracker::dtor_called = false;
  swap(ptr1, ptr2);
  EXPECT_FALSE(DestructorTracker::dtor_called);  // Nothing got destroyed.
  EXPECT_EQ(ptr1.get(), raw1);
  EXPECT_EQ(ptr2.get(), raw2);
}

// Tests the release method.
GTEST_TEST(CopyableUniquePtrTest, ReleaseTest) {
  cup<DestructorTracker> ptr;
  EXPECT_EQ(ptr.get(), nullptr);
  DestructorTracker* raw = nullptr;

  // Case 1: Release a null ptr.
  raw = ptr.release();
  EXPECT_EQ(ptr.get(), nullptr);
  EXPECT_EQ(raw, nullptr);

  // Case 2: Release non-null ptr.
  DestructorTracker* src;
  ptr.reset(src = new DestructorTracker(1));
  EXPECT_EQ(ptr.get(), src);
  DestructorTracker::dtor_called = false;
  raw = ptr.release();
  EXPECT_FALSE(DestructorTracker::dtor_called);  // Nothing destroyed.
  EXPECT_EQ(raw, src);
  delete src;
}

// Tests the stream value.
GTEST_TEST(CopyableUniquePtrTest, StreamTest) {
  stringstream ss;
  CloneOnly* raw = nullptr;
  ss << raw;
  std::string null_str = ss.str();
  ss.str(std::string());

  // Stream null pointer value.
  cup<CloneOnly> ptr;
  ss << ptr;
  EXPECT_EQ(ss.str(), null_str);
  ss.str(std::string());

  // Stream non-null value.
  ptr.reset(raw = new CloneOnly(1));
  ss << raw;
  std::string raw_str = ss.str();
  ss.str(std::string());
  ss << ptr;
  EXPECT_EQ(ss.str(), raw_str);
}

// Tests the == tests between copyable_unique_ptr and other entities.
GTEST_TEST(CopyableUniquePtrTest, EqualityTest) {
  CloneOnlyChildWithClone* raw;
  // This assigns the same pointer to *two* unique pointers. However, one
  // releases the pointer before going out of scope. This facilitates equality
  // testing for truly equal pointers.
  cup<CloneOnly> ptr(raw = new CloneOnlyChildWithClone(1));
  cup<CloneOnlyChildWithClone> child_ptr(raw);
  cup<CloneOnlyChildWithClone> other_ptr(new CloneOnlyChildWithClone(1));
  cup<CloneOnlyChildWithClone> child_null;
  cup<CloneOnly> parent_null;

  EXPECT_FALSE(ptr == parent_null);
  EXPECT_FALSE(ptr == child_null);
  EXPECT_FALSE(child_ptr == parent_null);
  EXPECT_FALSE(child_ptr == child_null);
  EXPECT_TRUE(parent_null == child_null);
  EXPECT_TRUE(ptr == child_ptr);
  EXPECT_FALSE(child_ptr == other_ptr);
  EXPECT_FALSE(ptr == other_ptr);
  EXPECT_FALSE(nullptr == ptr);
  EXPECT_FALSE(ptr == nullptr);

  // Don't try to call destructor twice.
  child_ptr.release();
}

// Tests the != tests between copyable_unique_ptr and other entities.
GTEST_TEST(CopyableUniquePtrTest, InEqualityTest) {
  CloneOnlyChildWithClone* raw;
  cup<CloneOnly> ptr(raw = new CloneOnlyChildWithClone(1));
  cup<CloneOnlyChildWithClone> child_ptr(raw);
  cup<CloneOnlyChildWithClone> other_ptr(new CloneOnlyChildWithClone(1));
  cup<CloneOnlyChildWithClone> child_null;
  cup<CloneOnly> parent_null;

  EXPECT_TRUE(ptr != parent_null);
  EXPECT_TRUE(ptr != child_null);
  EXPECT_TRUE(child_ptr != parent_null);
  EXPECT_TRUE(child_ptr != child_null);
  EXPECT_FALSE(parent_null != child_null);
  EXPECT_FALSE(ptr != child_ptr);
  EXPECT_TRUE(child_ptr != other_ptr);
  EXPECT_TRUE(ptr != other_ptr);
  EXPECT_TRUE(nullptr != ptr);
  EXPECT_TRUE(ptr != nullptr);

  // Don't try to call destructor twice.
  child_ptr.release();
}

// Tests the < and <= tests between copyable_unique_ptr and other entities.
GTEST_TEST(CopyableUniquePtrTest, LessThanTest) {
  CloneOnlyChildWithClone* raw;
  cup<CloneOnly> ptr(raw = new CloneOnlyChildWithClone(1));
  cup<CloneOnlyChildWithClone> child_ptr(raw);
  cup<CloneOnlyChildWithClone> other_ptr(new CloneOnlyChildWithClone(1));
  cup<CloneOnlyChildWithClone> child_null;
  cup<CloneOnly> parent_null;
  bool other_less_than_raw = other_ptr.get() < raw;

  EXPECT_FALSE(ptr < parent_null);
  EXPECT_FALSE(ptr < child_null);
  EXPECT_FALSE(child_ptr < parent_null);
  EXPECT_FALSE(child_ptr < child_null);
  EXPECT_FALSE(parent_null < child_null);
  EXPECT_TRUE(nullptr < ptr);
  EXPECT_FALSE(ptr < nullptr);
  EXPECT_FALSE(ptr < child_ptr);
  // I don't know which is actually less a priori. XORing with the counter test
  // guarantees that these should always evaluate to true in successful tests.
  EXPECT_TRUE((child_ptr < other_ptr) ^ other_less_than_raw);
  EXPECT_TRUE((ptr < other_ptr) ^ other_less_than_raw);

  EXPECT_FALSE(ptr <= parent_null);
  EXPECT_FALSE(ptr <= child_null);
  EXPECT_FALSE(child_ptr <= parent_null);
  EXPECT_FALSE(child_ptr <= child_null);
  EXPECT_TRUE(parent_null <= child_null);
  EXPECT_TRUE(nullptr <= ptr);
  EXPECT_FALSE(ptr <= nullptr);
  EXPECT_TRUE(ptr <= child_ptr);
  // I don't know which is actually less a priori. XORing with the counter test
  // guarantees that these should always evaluate to true in successful tests.
  EXPECT_TRUE((child_ptr <= other_ptr) ^ other_less_than_raw);
  EXPECT_TRUE((ptr <= other_ptr) ^ other_less_than_raw);

  // Don't try to call destructor twice.
  child_ptr.release();
}

// Tests the > and >= tests between copyable_unique_ptr and other entities.
GTEST_TEST(CopyableUniquePtrTest, GreaterThanTest) {
  CloneOnlyChildWithClone* raw;
  cup<CloneOnly> ptr(raw = new CloneOnlyChildWithClone(1));
  cup<CloneOnlyChildWithClone> child_ptr(raw);
  cup<CloneOnlyChildWithClone> other_ptr(new CloneOnlyChildWithClone(1));
  cup<CloneOnlyChildWithClone> child_null;
  cup<CloneOnly> parent_null;
  bool other_greater_than_raw = other_ptr.get() > raw;

  EXPECT_TRUE(ptr > parent_null);
  EXPECT_TRUE(ptr > child_null);
  EXPECT_TRUE(child_ptr > parent_null);
  EXPECT_TRUE(child_ptr > child_null);
  EXPECT_FALSE(parent_null > child_null);
  EXPECT_FALSE(nullptr > ptr);
  EXPECT_TRUE(ptr > nullptr);
  EXPECT_FALSE(ptr > child_ptr);
  // I don't know which is actually less a priori. XORing with the counter test
  // guarantees that these should always evaluate to true in successful tests.
  EXPECT_TRUE((child_ptr > other_ptr) ^ other_greater_than_raw);
  EXPECT_TRUE((ptr > other_ptr) ^ other_greater_than_raw);

  EXPECT_TRUE(ptr >= parent_null);
  EXPECT_TRUE(ptr >= child_null);
  EXPECT_TRUE(child_ptr >= parent_null);
  EXPECT_TRUE(child_ptr >= child_null);
  EXPECT_TRUE(parent_null >= child_null);
  EXPECT_FALSE(nullptr >= ptr);
  EXPECT_TRUE(ptr >= nullptr);
  EXPECT_TRUE(ptr >= child_ptr);
  // I don't know which is actually less a priori. XORing with the counter test
  // guarantees that these should always evaluate to true in successful tests.
  EXPECT_TRUE((child_ptr >= other_ptr) ^ other_greater_than_raw);
  EXPECT_TRUE((ptr >= other_ptr) ^ other_greater_than_raw);

  // Don't try to call destructor twice.
  child_ptr.release();
}

}  // namespace
}  // namespace drake
