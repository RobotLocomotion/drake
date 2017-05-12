#include "drake/common/singleton_lock.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

namespace drake {

namespace {

using std::make_shared;
using std::shared_ptr;

/*
 * This test verifies that the SingletonLock will scope access to a resource.
 */
int global_counter = 0;

/*
 * Simple resource to change a global counter when it is acquired / released.
 */
class Resource {
 public:
  Resource()
    : value_(++global_counter) {}
  ~Resource() {
    --global_counter;
  }
  int value() const { return value_; }
 private:
  int value_{};
};

/*
 * Incorporate expectations into resource lock class.
 */
class ResourceLock {
 public:
  explicit ResourceLock(int use_count_expected)
      : ResourceLock(use_count_expected, use_count_expected) {}

  ResourceLock(int use_count_ctor_expected, int use_count_dtor_expected,
               int value_expected = 1)
      : use_count_dtor_expected_(use_count_dtor_expected) {
    EXPECT_EQ(value_expected, lock_.instance().value());
    EXPECT_EQ(use_count_ctor_expected, lock_.use_count());
  }

  ~ResourceLock() {
    EXPECT_EQ(use_count_dtor_expected_, lock_.use_count());
  }

 private:
  SingletonLock<Resource> lock_;
  int use_count_dtor_expected_;
};

/*
 * Test neatly nested locks.
 */
GTEST_TEST(SingletonLock, NestedTest) {
  EXPECT_EQ(0, global_counter);
  {
    ResourceLock first(1);
    EXPECT_EQ(1, global_counter);
    {
      ResourceLock second(2);
      EXPECT_EQ(1, global_counter);
      {
        ResourceLock third(3);
        EXPECT_EQ(1, global_counter);
      }
      EXPECT_EQ(1, global_counter);
    }
    EXPECT_EQ(1, global_counter);
  }
  EXPECT_EQ(0, global_counter);
}

/*
 * Test ragged destruction with nesting mixed in.
 */
GTEST_TEST(SingletonLock, RaggedTest) {
  EXPECT_EQ(0, global_counter);

  {
    auto first = make_shared<ResourceLock>(1, 2);
    EXPECT_EQ(1, global_counter);

    auto second = make_shared<ResourceLock>(2, 3);
    EXPECT_EQ(1, global_counter);

    auto third = make_shared<ResourceLock>(3, 1);
    EXPECT_EQ(1, global_counter);

    {
      ResourceLock fourth(4);
      EXPECT_EQ(1, global_counter);
    }
    EXPECT_EQ(1, global_counter);

    second.reset();
    EXPECT_EQ(1, global_counter);

    first.reset();
    EXPECT_EQ(1, global_counter);
  }

  // Let third be implicitly destroyed
  EXPECT_EQ(0, global_counter);
}

/*
 * Incorporate expectations into resource lock class, but provide ability to
 * further specialize.
 */
template <typename Parent>
class SpecializedResourceLock {
 public:
  explicit SpecializedResourceLock(int use_count_expected)
      : SpecializedResourceLock(use_count_expected, use_count_expected) {}

  SpecializedResourceLock(int use_count_ctor_expected,
                          int use_count_dtor_expected,
                          int value_expected = 1)
      : use_count_dtor_expected_(use_count_dtor_expected) {
    EXPECT_EQ(value_expected, lock_.instance().value());
    EXPECT_EQ(use_count_ctor_expected, lock_.use_count());
  }

  ~SpecializedResourceLock() {
    EXPECT_EQ(use_count_dtor_expected_, lock_.use_count());
  }

 private:
  SingletonLock<Resource, Parent> lock_;
  int use_count_dtor_expected_;
};

struct A {};
struct B {};

/*
 * Test unique specializations for a Resource singleton.
 */
GTEST_TEST(SingletonLock, SpecializationTest) {
  EXPECT_EQ(0, global_counter);

  // These will be the same since it uses `void` (the default Parent argument).
  {
    SpecializedResourceLock<void> first(1);
    EXPECT_EQ(1, global_counter);
    ResourceLock second(2);
    EXPECT_EQ(1, global_counter);
  }
  EXPECT_EQ(0, global_counter);

  // This will be different.
  // Note that each singleton will record the value of global_counter:
  //   ResourceLock.instance().value == 1
  //   SpecializedResourceLock<A>.instance().value == 2
  //   SpecializedResourceLock<B>.instance().value == 3
  const int counter_normal = 1;
  const int counter_a = 2;
  const int counter_b = 3;
  {
    ResourceLock normal_1(1, 1, counter_normal);
    EXPECT_EQ(1, global_counter);

    {
      SpecializedResourceLock<A> a_1(1, 1, counter_a);
      EXPECT_EQ(2, global_counter);

      // Will refer to singleton value
      ResourceLock normal_2(2, 2, counter_normal);
      // Global value still remains
      EXPECT_EQ(2, global_counter);

      {
        SpecializedResourceLock<B> b_1(1, 1, counter_b);
        EXPECT_EQ(3, global_counter);

        SpecializedResourceLock<A> a_2(2, 2, counter_a);
        EXPECT_EQ(3, global_counter);

        SpecializedResourceLock<B> b_2(2, 2, counter_b);
        EXPECT_EQ(3, global_counter);
      }
      EXPECT_EQ(2, global_counter);
    }
    EXPECT_EQ(1, global_counter);
  }
  EXPECT_EQ(0, global_counter);
}

}  // anonymous namespace
}  // namespace drake
