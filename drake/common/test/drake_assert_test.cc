#include "drake/common/drake_assert.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace {

GTEST_TEST(DrakeAssertTest, AbortTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH(
      { DRAKE_ABORT(); },
      "abort: failure at .*drake_assert_test.cc:.. in TestBody");
}

GTEST_TEST(DrakeAssertTest, AbortUnlessTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH(
      { DRAKE_ABORT_UNLESS(false); },
      "abort: failure at .*drake_assert_test.cc:.. in TestBody:"
      " assertion 'false' failed");
}

struct BoolConvertible { operator bool() { return true; } };
GTEST_TEST(DrakeAssertTest, AssertSyntaxTest) {
  // These should compile.
  DRAKE_ASSERT((2 + 2) == 4);
  DRAKE_ASSERT(BoolConvertible());
}

// Only run this test if assertions are armed.
#ifdef DRAKE_ASSERT_IS_ARMED
GTEST_TEST(DrakeAssertTest, AssertFalseTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH(
      { DRAKE_ASSERT((2 + 2) == 5); },
      "abort: failure at .*drake_assert_test.cc:.. in TestBody: "
      "assertion '\\(2 \\+ 2\\) == 5' failed");
}
#endif  //  DRAKE_ASSERT_IS_ARMED

}  // namespace
