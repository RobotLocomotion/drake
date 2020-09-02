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

// Calls calloc (and then immediately frees).
void CallCalloc() {
  void* dummy = calloc(1, 16);
  g_dummy = dummy;
  free(dummy);
  if (g_dummy == nullptr) { throw std::runtime_error("null dummy"); }
}

// Calls realloc (and then immediately frees).
void CallRealloc() {
  void* dummy = realloc(nullptr, 16);
  g_dummy = dummy;
  free(dummy);
  if (g_dummy == nullptr) { throw std::runtime_error("null dummy"); }
}

// A value-parameterized test fixture.
class LimitMallocTest : public ::testing::TestWithParam<int> {
 public:
  void Allocate() {
    switch (GetParam()) {
      case 0: CallMalloc(); return;
      case 1: CallCalloc(); return;
      case 2: CallRealloc(); return;
    }
    throw std::logic_error("Bad GetParam()");
  }
};

// Use the same fixture for death tests, but with a different name.  Note that
// Drake's styleguide forbids death tests, but our only choice here is to use
// death tests because our implementations sense failure in contexts where
// exceptions are forbidden. The implementation of max limits must use abort()
// because exceptions cannot propagate through malloc() because it is a C
// ABI. Similarly, the implementation of min limits must use abort() because it
// sense failure within a destructor, which is a C++ `noexcept` context.
using LimitMallocDeathTest = LimitMallocTest;

TEST_P(LimitMallocTest, UnlimitedTest) {
  LimitMalloc guard(LimitMallocParams{ /* no limits specified */ });
  // EXPECT_EQ() inside the guard also allocates.
  EXPECT_EQ(guard.num_allocations(), 0);
  EXPECT_EQ(guard.params().max_num_allocations, -1);
  EXPECT_EQ(guard.params().min_num_allocations, -1);
  EXPECT_EQ(guard.params().ignore_realloc_noops, false);
  Allocate();  // Malloc is OK.
  Allocate();  // Malloc is OK.
  Allocate();  // Malloc is OK.
}

TEST_P(LimitMallocTest, BasicTest) {
  Allocate();  // Malloc is OK.
  LimitMallocParams args;
  {
    LimitMalloc guard;
    args = guard.params();
    // The guarded code would go here; malloc is NOT ok.
  }
  Allocate();  // Malloc is OK again.
  // Test contents of returned args outside the guard where EXPECT_EQ() may
  // allocate.
  EXPECT_EQ(args.max_num_allocations, 0);
  EXPECT_EQ(args.min_num_allocations, -1);
  EXPECT_EQ(args.ignore_realloc_noops, false);
}

constexpr const char* const kMaxDeathMessage =
    "abort due to malloc #[0-9]+ while"
    " max_num_allocations = [0-9]+ in effect";

constexpr const char* const kMinDeathMessage =
    "abort due to scope end with [0-9]+ mallocs while"
    " min_num_allocations = [0-9]+ in effect";

TEST_P(LimitMallocDeathTest, BasicTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  Allocate();  // Malloc is OK.
  ASSERT_DEATH({
      LimitMalloc guard;
      Allocate();  // Malloc is NOT ok.
    }, kMaxDeathMessage);
}

TEST_P(LimitMallocTest, MaxLimitTest) {
  {
    LimitMalloc guard({.max_num_allocations = 1});
    Allocate();  // Once is okay.
    // Another call here would fail.
  }
  Allocate();  // Malloc is OK again.
}

TEST_P(LimitMallocTest, MinLimitTest) {
  {
    LimitMalloc guard({.max_num_allocations = 5, .min_num_allocations = 1});
    Allocate();  // Once is necessary.
    // Fewer calls here would fail.
  }
}

TEST_P(LimitMallocDeathTest, MaxLimitTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  const auto expected_message =
      std::string("Once was okay\n") + kMaxDeathMessage;
  ASSERT_DEATH({
      LimitMalloc guard({.max_num_allocations = 1});
      Allocate();
      std::cerr << "Once was okay\n";
      // This is a convenient place to test that the getter returns a correct
      // count.  We must check within a unit test that is known to be disabled
      // via the BUILD file for platforms without a working implementation.
      EXPECT_EQ(guard.num_allocations(), 1);
      // A second allocation will fail.
      Allocate();
    }, expected_message);
}

TEST_P(LimitMallocDeathTest, MinLimitTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH({
      LimitMalloc guard({.max_num_allocations = 5, .min_num_allocations = 4});
      Allocate();  // Some few allocations are not enough.
      // The destructor will fail.
    }, kMinDeathMessage);
}

INSTANTIATE_TEST_SUITE_P(
    All, LimitMallocTest, ::testing::Range(0, 3));
INSTANTIATE_TEST_SUITE_P(
    All, LimitMallocDeathTest, ::testing::Range(0, 3));

// When the user whitelists no-op reallocs, a call to realloc() that does not
// change the size should not fail.
GTEST_TEST(LimitReallocTest, ChangingSizeTest) {
  void* dummy = malloc(16);;
  g_dummy = dummy;
  ASSERT_TRUE(g_dummy != nullptr);
  {
    LimitMalloc guard({
        .max_num_allocations = 0,
        .min_num_allocations = -1,
        .ignore_realloc_noops = true
    });
    dummy = realloc(dummy, 16);  // No change.
    g_dummy = dummy;
  }
  dummy = realloc(dummy, 16384);  // A change.
  g_dummy = dummy;
  free(dummy);
}

// When the user whitelists no-op reallocs, a call to realloc() should fail iff
// the size changed.
GTEST_TEST(LimitReallocDeathTest, ChangingSizeTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  void* dummy = malloc(16);;
  g_dummy = dummy;
  ASSERT_TRUE(g_dummy != nullptr);
  const auto expected_message =
      std::string("Once was okay\n") + kMaxDeathMessage;
  ASSERT_DEATH({
      LimitMalloc guard({
          .max_num_allocations = 0,
          .min_num_allocations = -1,
          .ignore_realloc_noops = true
      });
      dummy = realloc(dummy, 16);  // No change.
      g_dummy = dummy;
      std::cerr << "Once was okay\n";
      dummy = realloc(dummy, 16384);  // A change.
      g_dummy = dummy;
    }, expected_message);
  free(dummy);
}

}  // namespace
}  // namespace test
}  // namespace drake
