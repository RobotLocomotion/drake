#include "drake/common/temp_directory.h"

#include <cstdlib>
#include <string>

#include <gtest/gtest.h>
#include <spruce.hh>

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

  const char* tmpdir = std::getenv("TMPDIR");
  const int setenv_tmpdir_result_pre = ::setenv("TMPDIR", test_tmpdir, 1);
  ASSERT_EQ(0, setenv_tmpdir_result_pre);

  const int unset_test_tmpdir_result = ::unsetenv("TEST_TMPDIR");
  ASSERT_EQ(0, unset_test_tmpdir_result);

  spruce::path temp_directory_prefix(test_tmpdir);
  temp_directory_prefix.append("robotlocomotion_drake_");

  const std::string temp_directory_with_test_tmpdir_unset = temp_directory();
  EXPECT_EQ(0, temp_directory_with_test_tmpdir_unset.find(
      temp_directory_prefix.getStr()));

  const int setenv_test_tmpdir_result = ::setenv("TEST_TMPDIR", test_tmpdir, 1);
  ASSERT_EQ(0, setenv_test_tmpdir_result);

  if (tmpdir == nullptr) {
    const int unset_tmpdir_result = ::unsetenv("TMPDIR");
    ASSERT_EQ(0, unset_tmpdir_result);
  } else {
    const int setenv_tmpdir_result_post = ::setenv("TMPDIR", tmpdir, 1);
    ASSERT_EQ(0, setenv_tmpdir_result_post);
  }
}

}  // namespace
}  // namespace drake
