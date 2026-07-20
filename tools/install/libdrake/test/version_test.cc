#include "drake/version.h"

#include <gtest/gtest.h>

namespace drake {
namespace {

// Only packaging builds set a version stamp, and those do not run Bazel tests
// (nor install tests), so the version is always unstamped here.
GTEST_TEST(VersionAtLeastTest, BasicUnstamped) {
  EXPECT_FALSE(DRAKE_VERSION_AT_LEAST(0, 0, 0, 0));
  EXPECT_FALSE(DRAKE_VERSION_AT_LEAST(1, 51, 1, 20260311));
}

GTEST_TEST(VersionStringTest, Unstamped) {
  EXPECT_STREQ(DRAKE_VERSION_STRING, "unknown");
}

}  // namespace
}  // namespace drake
