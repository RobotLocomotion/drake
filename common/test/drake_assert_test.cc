#include "drake/common/drake_assert.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

GTEST_TEST(DrakeAssertTest, MatchingConfigTest) {
#ifdef DRAKE_ASSERT_IS_ARMED
  EXPECT_TRUE(::drake::kDrakeAssertIsArmed);
  EXPECT_FALSE(::drake::kDrakeAssertIsDisarmed);
#else
  EXPECT_FALSE(::drake::kDrakeAssertIsArmed);
  EXPECT_TRUE(::drake::kDrakeAssertIsDisarmed);
#endif
#ifdef DRAKE_ASSERT_IS_DISARMED
  EXPECT_FALSE(::drake::kDrakeAssertIsArmed);
  EXPECT_TRUE(::drake::kDrakeAssertIsDisarmed);
#else
  EXPECT_TRUE(::drake::kDrakeAssertIsArmed);
  EXPECT_FALSE(::drake::kDrakeAssertIsDisarmed);
#endif
}

// Note that Drake's styleguide forbids death tests, but our only choice here
// is to use death tests because our implementation is documented to abort().

GTEST_TEST(DrakeAssertDeathTest, DemandTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_DEATH(
      { DRAKE_DEMAND(false); },
      "abort: Failure at .*drake_assert_test.cc:.. in TestBody..: "
      "condition 'false' failed");
}

struct BoolConvertible { operator bool() const { return true; } };
GTEST_TEST(DrakeAssertDeathTest, AssertSyntaxTest) {
  // These should compile.
  DRAKE_ASSERT((2 + 2) == 4);
  DRAKE_ASSERT(BoolConvertible());
}

GTEST_TEST(DrakeAssertDeathTest, AssertFalseTest) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  if (::drake::kDrakeAssertIsArmed) {
    ASSERT_DEATH(
        { DRAKE_ASSERT(2 + 2 == 5); },
        "abort: Failure at .*drake_assert_test.cc:.. in TestBody..: "
        "condition '2 \\+ 2 == 5' failed");
  } else {
    DRAKE_ASSERT(2 + 2 == 5);
  }
}

GTEST_TEST(DrakeAssertDeathTest, AssertVoidTestArmed) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  if (::drake::kDrakeAssertIsArmed) {
    ASSERT_DEATH(
        { DRAKE_ASSERT_VOID(::abort()); },
        "");
  } else {
    DRAKE_ASSERT_VOID(::abort());
  }
}

}  // namespace
