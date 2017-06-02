#include "drake/common/scoped_singleton.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

namespace drake {

namespace {

using std::make_shared;
using std::shared_ptr;
using std::weak_ptr;

/*
 * A trivial singleton class that tracks the total number of instances alive.
 */
class InstanceCountedDummy {
 public:
  InstanceCountedDummy() {
    instance_count_++;
  }
  ~InstanceCountedDummy() {
    instance_count_--;
  }
  static int instance_count() { return instance_count_; }
 private:
  static int instance_count_;
};
int InstanceCountedDummy::instance_count_ = 0;

// Trivial type for specializing.
struct Specialized {};

/*
 * Test neatly nested locks.
 */
GTEST_TEST(ScopedSingletonTest, NestedAndSpecializedTest) {
  weak_ptr<InstanceCountedDummy> wref;
  EXPECT_EQ(0, wref.use_count());
  EXPECT_EQ(0, InstanceCountedDummy::instance_count());
  {
    auto ref_1 = GetScopedSingleton<InstanceCountedDummy>();
    wref = ref_1;
    EXPECT_EQ(1, wref.use_count());
    EXPECT_EQ(1, InstanceCountedDummy::instance_count());
    {
      auto ref_2 = GetScopedSingleton<InstanceCountedDummy>();
      EXPECT_EQ(2, wref.use_count());
      EXPECT_EQ(1, InstanceCountedDummy::instance_count());
      // Use specialized version.
      auto ref_a = GetScopedSingleton<InstanceCountedDummy, Specialized>();
      EXPECT_EQ(1, ref_a.use_count());
      // Original singleton's reference count should not change.
      EXPECT_EQ(2, wref.use_count());
      // We should have two unique InstanceCountedDummy instances.
      EXPECT_EQ(2, InstanceCountedDummy::instance_count());
    }
    EXPECT_EQ(1, wref.use_count());
    EXPECT_EQ(1, InstanceCountedDummy::instance_count());
  }
  EXPECT_EQ(0, wref.use_count());
  EXPECT_EQ(0, InstanceCountedDummy::instance_count());
}

}  // anonymous namespace
}  // namespace drake
