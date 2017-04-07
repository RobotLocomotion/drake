#include "drake/common/copyable_unique_ptr.h"

#include <memory>
#include <sstream>

#include <gtest/gtest.h>

#include "drake/common/test/is_dynamic_castable.h"

namespace drake {
namespace  {

using std::make_unique;
using std::move;
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
  Base(int v, Origin org=Origin::CONSTRUCT) : origin(org), value(v) {}
  virtual ~Base() {}

  Origin origin{Origin::UNINITIALIZED};
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
  EXPECT_TRUE(cup<CopyOnly>::is_copyable::value);
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
  EXPECT_TRUE(cup<FullyCopyable>::is_copyable::value);
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
  ~CloneOnly() { value = -1; }
  CloneOnly(const CloneOnly&) = delete;
  unique_ptr<CloneOnly> Clone() const {
    return make_unique<CloneOnly>(value, Origin::CLONE);
  }
};

// Confirms that a class that has a proper implementation of Clone, but no
// copy constructor clones itself.
GTEST_TEST(CopyableUniquePtrTest, CloneOnlySuccess) {
  EXPECT_TRUE(cup<CloneOnly>::is_copyable::value);
  cup<CloneOnly> ptr(new CloneOnly(1));
  EXPECT_EQ(ptr->origin, Origin::CONSTRUCT);
  cup<CloneOnly> copy(ptr);
  EXPECT_EQ(copy->origin, Origin::CLONE);
  EXPECT_EQ(copy->value, ptr->value);
  copy->value++;
  EXPECT_NE(copy->value, ptr->value);
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
  EXPECT_TRUE(std::is_copy_constructible<FullyCopyable>::value);
  EXPECT_TRUE(cup<FullyCopyable>::is_copyable::value);

  // Case 2) True || False --> True
  EXPECT_TRUE(is_cloneable<CloneOnly>::value);
  EXPECT_FALSE(std::is_copy_constructible<CloneOnly>::value);
  EXPECT_TRUE(cup<CloneOnly>::is_copyable::value);

  // Case 3) False || True --> True
  EXPECT_FALSE(is_cloneable<CopyOnly>::value);
  EXPECT_TRUE(std::is_copy_constructible<CopyOnly>::value);
  EXPECT_TRUE(cup<CopyOnly>::is_copyable::value);

  // Case 4) False || False --> False
  EXPECT_FALSE(is_cloneable<Base>::value);
  EXPECT_FALSE(std::is_copy_constructible<Base>::value);
  EXPECT_FALSE(cup<Base>::is_copyable::value);
}

// -----------------------------------------------------------------------
// These tests cover the aspects of the copyable_unique_ptr that mirror
// unique_ptr functionality. No copying takes place.

// This tests the various constructor methods that create a null pointer. This
// implicitly relies on the empty() method.
GTEST_TEST(CopyableUniquePtrTest, ConstructNullPtr) {
  // Default constructor.
  cup<CopyOnly> ptr;
  EXPECT_TRUE(ptr.empty());

  // Explicitly null.
  cup<CopyOnly> ptr2(nullptr);
  EXPECT_TRUE(ptr2.empty());
}

// A class that derives from a copyable class.
struct CloneOnlyChild : CloneOnly {
  CloneOnlyChild(int v, Origin org=Origin::CONSTRUCT) : CloneOnly(v, org) {}
  CloneOnlyChild(const CloneOnlyChild&) = delete;
  unique_ptr<CloneOnlyChild> Clone() const {
    return make_unique<CloneOnlyChild>(value, Origin::CLONE);
  }
};

// Test initialization with non-null pointer. Covers same type and compatible
// type. This test implicitly relies on the get() method.
GTEST_TEST(CopyableUniquePtrTest, ConstructValidPtr) {
  CloneOnly* base_ptr;
  cup<CloneOnly> ptr(base_ptr = new CloneOnly(1));
  EXPECT_EQ(ptr.get(), base_ptr);

  CloneOnlyChild* co_ptr;
  cup<CloneOnly> ptr2(co_ptr = new CloneOnlyChild(2));
  ASSERT_TRUE(is_dynamic_castable<CloneOnlyChild>(ptr2.get()));
  EXPECT_EQ(ptr2.get(), co_ptr);
}

// Confirms accessor functionality for getting const and non-const pointers.
GTEST_TEST(CopyableUniquePtrTest, PointerAccessorSemantics) {
  cup<CloneOnly> ptr(new CloneOnly(1));

  // The extra parentheses prevent the macro from getting confused.
  // get is const pointer
  EXPECT_TRUE(
  (std::is_same<const CloneOnly*, decltype(ptr.get())>::value));

  // get_mutable is non-const
  EXPECT_TRUE((std::is_same<CloneOnly*, decltype(ptr.get_mutable())>::value));
}

// Confirms accessor functionality for getting const and non-const references.
GTEST_TEST(CopyableUniquePtrTest, RefAccessorSemantics) {
  cup<CloneOnly> ptr(new CloneOnly(1));

  // The extra parentheses prevent the macro from getting confused.
  // get is const pointer
  EXPECT_TRUE((std::is_same<const CloneOnly&, decltype(ptr.get_ref())>::value));

  // get_mutable is non-const
  EXPECT_TRUE(
      (std::is_same<CloneOnly&, decltype(ptr.get_mutable_ref())>::value));
}

// This class allows me to track destruction of instances in copyable_unique_ptr
// instances. To use it, set the static member live_state, and then perform an
// operation. If an instance has been destroyed, live_state will be false,
// otherwise, no destructor was called.
struct DestructorTracker : public CloneOnly {
  DestructorTracker(int v, Origin org=Origin::CONSTRUCT) : CloneOnly(v, org) {}
  virtual ~DestructorTracker() { live_state = false; }
  DestructorTracker(const DestructorTracker&) = delete;
  unique_ptr<DestructorTracker> Clone() const {
    return unique_ptr<DestructorTracker>(DoClone());
  }
  static bool live_state;
 protected:
  virtual DestructorTracker* DoClone() const {
    return new DestructorTracker(value, Origin::CLONE);
  }
};
bool DestructorTracker::live_state = true;

// Confirms destructor works as pointer goes out of scope.
GTEST_TEST(CopyableUniquePtrTest, Destructor) {
  // Empty copyable_unique_ptr does not invoke destructor.
  {
    cup<DestructorTracker> empty;
    DestructorTracker::live_state = true;
  }
  EXPECT_TRUE(DestructorTracker::live_state);

  // Non-empty pointer invokes destructor.
  DestructorTracker* raw_ptr = new DestructorTracker(1);
  {
    cup<DestructorTracker> ptr(raw_ptr);
    EXPECT_EQ(ptr.get(), raw_ptr);
    DestructorTracker::live_state = true;
  }
  EXPECT_FALSE(DestructorTracker::live_state);
}

// This tests the various incarnations of reset.
GTEST_TEST(CopyableUniquePtrTest, Reset) {
  cup<DestructorTracker> ptr;
  EXPECT_TRUE(ptr.empty());  // Confirm initial condition.

  // Case 1: Resetting an empty pointer invokes no constructor.
  DestructorTracker* raw_ptr = new DestructorTracker(1);
  DestructorTracker::live_state = true;
  ptr.reset(raw_ptr);
  EXPECT_EQ(ptr.get(), raw_ptr);
  // Previously empty pointer does *not* invoke destructor.
  EXPECT_TRUE(DestructorTracker::live_state);

  // Case 2: Previous non-null contents *are* destroyed.
  raw_ptr = new DestructorTracker(2);
  EXPECT_NE(ptr.get(), raw_ptr);
  DestructorTracker::live_state = true;
  ptr.reset(raw_ptr);
  EXPECT_EQ(ptr.get(), raw_ptr);
  // Previously empty pointer *does* invoke destructor.
  EXPECT_FALSE(DestructorTracker::live_state);

  // Case 3: Previous non-null contents replace by nullptr.
  DestructorTracker::live_state = true;
  ptr.reset();
  EXPECT_EQ(ptr.get(), nullptr);
  // Previously empty pointer *does* invoke destructor.
  EXPECT_FALSE(DestructorTracker::live_state);
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

  // The swap method.
  DestructorTracker::live_state = true;
  ptr1.swap(ptr2);
  EXPECT_TRUE(DestructorTracker::live_state);  // Nothing got destroyed.
  EXPECT_EQ(ptr1.get(), raw2);
  EXPECT_EQ(ptr2.get(), raw1);

  // The swap function.
  DestructorTracker::live_state = true;
  swap(ptr1, ptr2);
  EXPECT_TRUE(DestructorTracker::live_state);  // Nothing got destroyed.
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
  DestructorTracker::live_state = true;
  raw = ptr.release();
  EXPECT_TRUE(DestructorTracker::live_state);  // Nothing destroyed.
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
  CloneOnlyChild* raw;
  cup<CloneOnly> ptr(raw = new CloneOnlyChild(1));
  cup<CloneOnlyChild> child_ptr(raw);
  cup<CloneOnlyChild> other_ptr(new CloneOnlyChild(1));
  cup<CloneOnlyChild> child_null;
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
  CloneOnlyChild* raw;
  cup<CloneOnly> ptr(raw = new CloneOnlyChild(1));
  cup<CloneOnlyChild> child_ptr(raw);
  cup<CloneOnlyChild> other_ptr(new CloneOnlyChild(1));
  cup<CloneOnlyChild> child_null;
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
  CloneOnlyChild* raw;
  cup<CloneOnly> ptr(raw = new CloneOnlyChild(1));
  cup<CloneOnlyChild> child_ptr(raw);
  cup<CloneOnlyChild> other_ptr(new CloneOnlyChild(1));
  cup<CloneOnlyChild> child_null;
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
  CloneOnlyChild* raw;
  cup<CloneOnly> ptr(raw = new CloneOnlyChild(1));
  cup<CloneOnlyChild> child_ptr(raw);
  cup<CloneOnlyChild> other_ptr(new CloneOnlyChild(1));
  cup<CloneOnlyChild> child_null;
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

// This tests the move semantics. This can manifest either in initialization or
// in assignment.
GTEST_TEST(CopyableUniquePtrTest, MoveSemantics) {
  DestructorTracker* raw;
  cup<DestructorTracker> ptr(raw = new DestructorTracker(1));
  EXPECT_EQ(ptr.get(), raw);

  // Do not re-order the following tests, they depend on execution order for
  // correctness.

  // Initialization from rvalue copyable_unique_ptr.
  DestructorTracker::live_state = true;
  cup<DestructorTracker> ptr1(move(ptr));
  EXPECT_TRUE(DestructorTracker::live_state);
  EXPECT_EQ(ptr.get(), nullptr);
  EXPECT_EQ(ptr1.get(), raw);

  // Assignment from rvalue copyable_unique_ptr.
  cup<DestructorTracker> ptr2;
  DestructorTracker::live_state = true;
  ptr2 = move(ptr1);
  EXPECT_TRUE(DestructorTracker::live_state);
  EXPECT_EQ(ptr1.get(), nullptr);
  EXPECT_EQ(ptr2.get(), raw);
}

// This tests the move semantics of an object into a copyable_unique_ptr of an
// ancestor class type. This test doesn't test for destruction (it relies on
// MoveSemantics test to have already covered that.
GTEST_TEST(CopyableUniquePtrTest, MoveDerivedSemantics) {
  CloneOnlyChild* raw;
  cup<CloneOnlyChild> ptr(raw = new CloneOnlyChild(1));
  EXPECT_EQ(ptr.get(), raw);

  // Do not re-order the following tests, they depend on execution order for
  // correctness.

  // Initialization from rvalue copyable_unique_ptr.
  cup<CloneOnly> ptr1(move(ptr));
  EXPECT_EQ(ptr.get(), nullptr);
  EXPECT_EQ(ptr1.get(), raw);
  ptr1.release();
  ptr.reset(raw);

  // Assignment from rvalue copyable_unique_ptr.
  cup<CloneOnly> ptr2;
  ptr2 = move(ptr);
  EXPECT_EQ(ptr.get(), nullptr);
  EXPECT_EQ(ptr2.get(), raw);
}

// This class allows detecting destruction of derived class relative to a parent
// class.
struct DestructorChild : public DestructorTracker {
  DestructorChild(int v, Origin org = Origin::CONSTRUCT) :
      DestructorTracker(v, org) {}
  DestructorChild(const DestructorChild&) = delete;

 protected:
  DestructorChild* DoClone() const override {
    return new DestructorChild(value, Origin::CLONE);
  }
};

// Tests the simple assignment operator - assignment of pointer.
GTEST_TEST(CopyableUniquePtrTest, RawPointerAssignmentTest) {
  DestructorTracker *d_raw = new DestructorTracker(1);
  cup<DestructorTracker> d_ptr;

  // Case 1: Assignment of same type pointer into a null pointer.
  DestructorTracker::live_state = true;
  d_ptr = d_raw;
  EXPECT_TRUE(DestructorTracker::live_state);
  EXPECT_EQ(d_ptr.get(), d_raw);

  // Case 2: Assignment of same type pointer into a non-null pointer.
  d_raw = new DestructorTracker(2);  // Grab a new pointer.
  DestructorTracker::live_state = true;
  d_ptr = d_raw;
  EXPECT_FALSE(DestructorTracker::live_state);
  EXPECT_EQ(d_ptr.get(), d_raw);

  d_ptr.reset();
  EXPECT_EQ(d_ptr.get(), nullptr);

  // Case 3: Assignment of derived type pointer into a null pointer.
  DestructorChild* c_raw = new DestructorChild(3);
  DestructorTracker::live_state = true;
  d_ptr = c_raw;
  EXPECT_TRUE(DestructorTracker::live_state);
  EXPECT_EQ(d_ptr.get(), c_raw);

  // Case 4: Assignment of derived type pointer into a non-null pointer.
  c_raw = new DestructorChild(4);
  DestructorTracker::live_state = true;
  d_ptr = c_raw;
  EXPECT_FALSE(DestructorTracker::live_state);
  EXPECT_EQ(d_ptr.get(), c_raw);
}

// Tests the case where a unique_ptr is assigned to by a reference.
GTEST_TEST(CopyableUniquePtrTest, RefAssignmentTest) {
  DestructorTracker data(1325);
  cup<DestructorTracker> ptr;
  EXPECT_EQ(ptr.get(), nullptr);

  // Case 1: Assign to null pointer.
  DestructorTracker::live_state = true;
  ptr = data;
  EXPECT_TRUE(DestructorTracker::live_state);
  EXPECT_NE(ptr.get(), nullptr);
  EXPECT_NE(ptr.get(), &data);
  EXPECT_EQ(ptr->value, data.value);
  data.value = 3;
  const DestructorTracker* old_ptr = ptr.get();

  // Case 2: Assign to non-null pointer.
  DestructorTracker::live_state = true;
  ptr = data;
  EXPECT_FALSE(DestructorTracker::live_state);
  EXPECT_NE(ptr.get(), old_ptr);
  EXPECT_NE(ptr.get(), &data);
  EXPECT_EQ(ptr->value, data.value);
}

// Tests the case where a unique_ptr is assigned to by another
// copyable_unique_ptr.
GTEST_TEST(CopyableUniquePtrTest, RefPtrAssignmentTest) {
  DestructorTracker* raw;
  cup<DestructorTracker> src(raw = new DestructorTracker(1352));
  cup<DestructorTracker> ptr;
  EXPECT_EQ(ptr.get(), nullptr);

  // Case 1: Assign to null pointer.
  DestructorTracker::live_state = true;
  ptr = src;
  EXPECT_TRUE(DestructorTracker::live_state);
  EXPECT_NE(ptr.get(), raw);
  EXPECT_EQ(ptr->value, raw->value);
  EXPECT_EQ(src.get(), raw);
  raw->value = 1111;

  // Case 2: Assign to non-null pointer.
  DestructorTracker::live_state = true;
  ptr = src;
  EXPECT_FALSE(DestructorTracker::live_state);
  EXPECT_NE(ptr.get(), raw);
  EXPECT_EQ(ptr->value, raw->value);
  EXPECT_EQ(src.get(), raw);

  // Tests with pointers to derived classes.
  DestructorChild* child_raw;
  cup<DestructorChild> child_src(child_raw = new DestructorChild(15));
  ptr.reset();

  // Case 3: Assign derived class to null pointer.
  DestructorTracker::live_state = true;
  ptr = child_src;
  EXPECT_TRUE(DestructorTracker::live_state);
  EXPECT_NE(ptr.get(), child_raw);
  EXPECT_EQ(ptr->value, child_raw->value);
  EXPECT_TRUE(is_dynamic_castable<DestructorChild>(ptr.get()));
  EXPECT_NE(dynamic_cast<const DestructorTracker*>(ptr.get()), nullptr);
  EXPECT_EQ(child_src.get(), child_raw);
  raw->value = 32;

  // Case 4: Assign derived to non-null pointer.
  DestructorTracker::live_state = true;
  ptr = child_src;
  EXPECT_FALSE(DestructorTracker::live_state);
  EXPECT_NE(ptr.get(), child_raw);
  EXPECT_EQ(ptr->value, child_raw->value);
  EXPECT_TRUE(is_dynamic_castable<DestructorChild>(ptr.get()));
  EXPECT_EQ(child_src.get(), child_raw);
}

// This tests the constructors that generate copies of the input (i.e., from
// a const pointer or a const reference)
GTEST_TEST(CopyableUniquePtrTest, ConstructorCopyTest) {
  CloneOnly src(10);
  cup<CloneOnly> from_ref(src);
  EXPECT_NE(from_ref.get(), &src);
  EXPECT_EQ(from_ref->value, src.value);
  src.value = 20;
  EXPECT_NE(from_ref->value, src.value);

  const CloneOnly* src_ptr = &src;
  cup<CloneOnly> from_ptr(src_ptr);
  EXPECT_NE(from_ptr.get(), &src);
  EXPECT_EQ(from_ptr->value, src.value);
  src.value = 30;
  EXPECT_NE(from_ptr->value, src.value);

  from_ptr.release();  // Don't call destructor twice
}

// Suite of tests to catch the debug-build assertions.
#ifndef DRAKE_ASSERT_IS_DISARMED

// This confirms in debug build, that accessing a null pointer value for a const
// reference aborts as expected.
GTEST_TEST(CopyableUniquePtrDeathTest, NullConstAccessorSemantics) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  cup<CloneOnly> ptr{nullptr};
  ASSERT_EQ(ptr.get(), nullptr);
  ASSERT_DEATH(
      {ptr.get_ref();},
      "abort: failure at .*copyable_unique_ptr.h:.+ in get.+"
          "assertion '!empty\\(\\)' failed");
}

// This confirms in debug build, that accessing a null pointer value for a
// mutable reference aborts as expected.
GTEST_TEST(CopyableUniquePtrDeathTest, NullMutableAccessorSemantics) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  cup<CloneOnly> ptr{nullptr};
  ASSERT_EQ(ptr.get(), nullptr);
  ASSERT_DEATH(
      {ptr.get_mutable_ref();},
      "abort: failure at .*copyable_unique_ptr.h:.+ in get_mutable.+"
          "assertion '!empty\\(\\)' failed");
}

#endif  // DRAKE_ASSERT_IS_DISARMED
}  // namespace
}  // namespace drake
