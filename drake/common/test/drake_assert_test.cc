#include "drake/common/drake_assert.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace {

GTEST_TEST(DrakeAssertDeathTest, AbortTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH(
      { DRAKE_ABORT(); },
      "abort: failure at .*drake_assert_test.cc:.. in TestBody");
}

GTEST_TEST(DrakeAssertDeathTest, DemandTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH(
      { DRAKE_DEMAND(false); },
      "abort: failure at .*drake_assert_test.cc:.. in TestBody..:"
      " assertion 'false' failed");
}

struct BoolConvertible { operator bool() const { return true; } };
GTEST_TEST(DrakeAssertDeathTest, AssertSyntaxTest) {
  // These should compile.
  DRAKE_ASSERT((2 + 2) == 4);
  DRAKE_ASSERT(BoolConvertible());
}

// Only run these tests if assertions are armed.
#ifdef DRAKE_ASSERT_IS_ARMED

GTEST_TEST(DrakeAssertDeathTest, AssertFalseTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH(
      { DRAKE_ASSERT((2 + 2) == 5); },
      R"(abort: failure at .*drake_assert_test.cc:.. in TestBody\(\): )"
      R"(assertion '\(2 \+ 2\) == 5' failed)");
}

GTEST_TEST(DrakeAssertDeathTest, AssertVoidTestArmed) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH(
      { DRAKE_ASSERT_VOID(::abort()); },
      R"()");
}

#endif  //  DRAKE_ASSERT_IS_ARMED

}  // namespace
