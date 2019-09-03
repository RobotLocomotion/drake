#include <stdlib.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_runfiles.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace {

// Test error message codepaths.  We purposefully lobotomize the environment
// variables before latch-initializing the runfiles singleton.  Because we
// can't reinitialize it, we can only write a single unit test in this file and
// we need to keep this case separate from the find_runfiles_test.cc cases.
GTEST_TEST(FindRunfilesFailTest, ErrorMessageTest) {
  drake::logging::set_log_level("trace");
  ASSERT_EQ(::setenv("TEST_SRCDIR", "/no_such_srcdir", 1), 0);
  ASSERT_EQ(::unsetenv("RUNFILES_DIR"), 0);
  ASSERT_EQ(::unsetenv("RUNFILES_MANIFEST_FILE"), 0);

  EXPECT_FALSE(HasRunfiles());
  const auto result = FindRunfile("drake/nothing");
  EXPECT_EQ(result.abspath, "");
  EXPECT_THAT(result.error, testing::MatchesRegex(
      "ERROR: .* cannot find runfiles .*"
      "created using TEST_SRCDIR with TEST_SRCDIR=/no_such_srcdir "
      "and RUNFILES_MANIFEST_FILE=nullptr and RUNFILES_DIR=nullptr."));
}

}  // namespace
}  // namespace drake
