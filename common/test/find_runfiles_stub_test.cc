/* clang-format off to disable clang-format-includes */
#include "drake/common/find_runfiles.h"
/* clang-format on */

#include <gtest/gtest.h>

namespace drake {
namespace {

// Checks that find_runfiles_stub successfully compiles and links.
GTEST_TEST(FindRunfilesTest, AcceptanceTest) {
  EXPECT_FALSE(HasRunfiles());
  const auto result = FindRunfile("drake/foo");
  EXPECT_EQ(result.abspath, "");
  EXPECT_NE(result.error, "");
}

}  // namespace
}  // namespace drake
