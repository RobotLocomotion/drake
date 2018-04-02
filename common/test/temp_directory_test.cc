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

GTEST_TEST(TempDirectoryTest, TestTmpdirUnset) {
  const char* test_tmpdir = std::getenv("TEST_TMPDIR");
  ASSERT_STRNE(nullptr, test_tmpdir);

  const int unset_result = ::unsetenv("TEST_TMPDIR");
  ASSERT_EQ(0, unset_result);

  const std::string temp_directory_with_test_tmpdir_unset = temp_directory();
  EXPECT_EQ("/tmp", temp_directory_with_test_tmpdir_unset);

  const int setenv_result = ::setenv("TEST_TMPDIR", test_tmpdir, 1);
  ASSERT_EQ(0, setenv_result);
}

}  // namespace
}  // namespace drake
