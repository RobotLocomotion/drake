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

// Use the same fixture for death tests, but with a different name.
using LimitMallocDeathTest = LimitMallocTest;

TEST_P(LimitMallocTest, UnlimitedTest) {
  LimitMalloc guard(LimitMallocParams{ /* no limits specified */ });
  Allocate();  // Malloc is OK.
  Allocate();  // Malloc is OK.
  Allocate();  // Malloc is OK.
}

TEST_P(LimitMallocTest, BasicTest) {
  Allocate();  // Malloc is OK.
  {
    LimitMalloc guard;
    // The guarded code would go here; malloc is NOT ok.
  }
  Allocate();  // Malloc is OK again.
}

constexpr const char* const kDeathMessage =
    "abort due to malloc while LimitMalloc is in effect";

TEST_P(LimitMallocDeathTest, BasicTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  Allocate();  // Malloc is OK.
  ASSERT_DEATH({
      LimitMalloc guard;
      Allocate();  // Malloc is NOT ok.
    }, kDeathMessage);
}

TEST_P(LimitMallocTest, LimitTest) {
  {
    LimitMalloc guard({.max_num_allocations = 1});
    Allocate();  // Once is okay.
    // Another call here would fail.
  }
  Allocate();  // Malloc is OK again.
}

TEST_P(LimitMallocDeathTest, LimitTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  const auto expected_message =
      std::string("Once was okay\n") + kDeathMessage;
  ASSERT_DEATH({
      LimitMalloc guard({.max_num_allocations = 1});
      Allocate();
      std::cerr << "Once was okay\n";
      Allocate();
    }, expected_message);
}

INSTANTIATE_TEST_CASE_P(
    All, LimitMallocTest, ::testing::Range(0, 3));
INSTANTIATE_TEST_CASE_P(
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
      std::string("Once was okay\n") + kDeathMessage;
  ASSERT_DEATH({
      LimitMalloc guard({
          .max_num_allocations = 0,
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
