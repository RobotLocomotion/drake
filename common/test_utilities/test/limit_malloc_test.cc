#include "drake/common/test_utilities/limit_malloc.h"

#include <cstdlib>
#include <stdexcept>

#include <gtest/gtest.h>

// A global variable to help prevent the compiler from optimizing out the call
// to malloc.  (Without this, it can reason that malloc-free is a no-op.)
volatile void* g_dummy;

namespace drake {
namespace test {
namespace {

// Calls malloc (and then immediately frees).
void CallMalloc() {
  void* dummy = malloc(16);
  g_dummy = dummy;
  free(dummy);
  if (g_dummy == nullptr) { throw std::runtime_error("null dummy"); }
}

GTEST_TEST(LimitMallocTest, UnlimitedTest) {
  LimitMalloc guard(LimitMallocParams{ /* no limits specified */ });
  CallMalloc();  // Malloc is OK.
  CallMalloc();  // Malloc is OK.
  CallMalloc();  // Malloc is OK.
}

GTEST_TEST(LimitMallocTest, BasicTest) {
  CallMalloc();  // Malloc is OK.
  {
    LimitMalloc guard;
    // The guarded code would go here; malloc is NOT ok.
  }
  CallMalloc();  // Malloc is OK again.
}

GTEST_TEST(LimitMallocDeathTest, BasicTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  CallMalloc();  // Malloc is OK.
  ASSERT_DEATH({
      LimitMalloc guard;
      CallMalloc();  // Malloc is NOT ok.
    }, "abort due to malloc while LimitMalloc is in effect");
}

GTEST_TEST(LimitMallocTest, LimitTest) {
  {
    LimitMalloc guard({.max_num_allocations = 1});
    CallMalloc();  // Once is okay.
    // Another call here would fail.
  }
  CallMalloc();  // Malloc is OK again.
}

GTEST_TEST(LimitMallocDeathTest, LimitTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH({
      LimitMalloc guard({.max_num_allocations = 1});
      CallMalloc();
      std::cerr << "Once was okay\n";
      CallMalloc();
    },
    "Once was okay\n"
    "abort due to malloc while LimitMalloc is in effect");
}

}  // namespace
}  // namespace test
}  // namespace drake
