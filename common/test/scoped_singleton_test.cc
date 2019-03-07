#include "drake/common/scoped_singleton.h"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/never_destroyed.h"

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
    const int momentary_total = ++mutable_instance_count();
    DRAKE_DEMAND(momentary_total <= max_instance_count());
  }

  ~InstanceCountedDummy() {
    const int momentary_total = --mutable_instance_count();
    DRAKE_DEMAND(momentary_total >= 0);
  }

  static int instance_count() {
    return mutable_instance_count();
  }

  static int max_instance_count() {
    return mutable_max_instance_count();
  }

  static std::atomic<int>& mutable_instance_count() {
    static never_destroyed<std::atomic<int>> global_counter(0);
    return global_counter.access();
  }

  static std::atomic<int>& mutable_max_instance_count() {
    static never_destroyed<std::atomic<int>> global_counter(1);
    return global_counter.access();
  }
};

// Trivial type for specializing.
struct Specialized {};

/*
 * Test neatly nested locks.
 */
GTEST_TEST(ScopedSingletonTest, NestedAndSpecializedTest) {
  InstanceCountedDummy::mutable_max_instance_count() = 2;
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

GTEST_TEST(ScopedSingletonTest, ThreadedTest) {
  // Abort if a violation of the singleton requirement is detected.
  InstanceCountedDummy::mutable_max_instance_count() = 1;
  EXPECT_EQ(InstanceCountedDummy::instance_count(), 0);

  // In each thread, ask for a singleton reference and then sleep.
  auto thread_action = []() {
    auto handle = GetScopedSingleton<InstanceCountedDummy>();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  };

  // Launch many threads as quickly as possible, each doing thread_action.
  std::vector<std::thread> threads;
  for (int i = 0; i < 20; ++i) {
    threads.emplace_back(thread_action);
  }

  // Wait for them all to finish.
  for (auto& item : threads) {
    item.join();
  }
  EXPECT_EQ(InstanceCountedDummy::instance_count(), 0);
}

}  // anonymous namespace
}  // namespace drake
