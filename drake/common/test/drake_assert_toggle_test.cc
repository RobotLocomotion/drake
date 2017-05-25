#include "drake/common/drake_assert_toggle.h"

#include <stdexcept>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"

namespace {

GTEST_TEST(DrakeAssertToggleTest, CatchTest) {
  drake::assert::set_assertion_failure_to_throw_exception();
  bool regex_test_was_run = false;
  EXPECT_THROW(
    try {
      // Use a DEMAND so that behavior is the same no matter the build variant.
      // By code inspection, we see that ASSERT and DEMAND share an execution
      // path, so we only need to test DEMAND.
      DRAKE_DEMAND(false);
    } catch (const std::exception& e) {
      // Confirm the error message.
      EXPECT_THAT(e.what(), testing::MatchesRegex(
          "Failure.*in TestBody.* condition 'false' failed."));
      regex_test_was_run = true;
      throw;
    }, std::exception);
  EXPECT_TRUE(regex_test_was_run);
}

}  // namespace
