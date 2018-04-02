#include "drake/common/temp_directory.h"

#include <cstdlib>
#include <string>

#include <gtest/gtest.h>

namespace drake {
namespace {

GTEST_TEST(TempDirectoryTest, TestTmpdirSet) {
  const char* test_tmpdir = std::getenv("TEST_TMPDIR");
  ASSERT_STRNE(nullptr, test_tmpdir);

  const std::string temp_directory_with_test_tmpdir_set = temp_directory();
  EXPECT_NE('/', temp_directory_with_test_tmpdir_set.back());
  EXPECT_EQ(std::string(test_tmpdir), temp_directory_with_test_tmpdir_set);
}

// We do not test the case when TEST_TMPDIR is unset since it would create a
// file /tmp/robotlocomotion_drake_XXXXXX outside the sandbox.

}  // namespace
}  // namespace drake
