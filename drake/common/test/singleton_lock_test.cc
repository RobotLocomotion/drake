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

  ResourceLock(int use_count_ctor_expected, int use_count_dtor_expected)
      : use_count_dtor_expected_(use_count_dtor_expected) {
    EXPECT_EQ(1, lock_.instance().value());
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

}  // anonymous namespace
}  // namespace drake
