#include "drake/common/temp_directory.h"

#include <cstdlib>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/filesystem.h"

namespace drake {
namespace {

GTEST_TEST(TempDirectoryTest, TestTmpdirSet) {
  const char* test_tmpdir = std::getenv("TEST_TMPDIR");
  ASSERT_STRNE(nullptr, test_tmpdir);

  const std::string temp_directory_with_test_tmpdir_set = temp_directory();
  EXPECT_NE('/', temp_directory_with_test_tmpdir_set.back());
  filesystem::path temp_path(temp_directory_with_test_tmpdir_set);
  EXPECT_EQ(std::string(test_tmpdir), temp_path.parent_path().string());
  EXPECT_THAT(temp_path.filename().string(),
              testing::MatchesRegex("robotlocomotion_drake_[^/]{6}$"));
}

GTEST_TEST(TempDirectoryTest, TestTmpdirUnset) {
  // Temporarily set TMP to TEST_TMPDIR and unset TEST_TMPDIR, and then see
  // what temp_directory() returns.
  const char* tmpdir = std::getenv("TMPDIR");
  const char* test_tmpdir = std::getenv("TEST_TMPDIR");
  ASSERT_STRNE(nullptr, test_tmpdir);
  DRAKE_DEMAND(::setenv("TMPDIR", test_tmpdir, 1) == 0);
  DRAKE_DEMAND(::unsetenv("TEST_TMPDIR") == 0);
  const std::string temp_directory_result = temp_directory();

  // Revert the environment changes.
  DRAKE_DEMAND(::setenv("TEST_TMPDIR", test_tmpdir, 1) == 0);
  if (tmpdir == nullptr) {
    DRAKE_DEMAND(::unsetenv("TMPDIR") == 0);
  } else {
    DRAKE_DEMAND(::setenv("TMPDIR", tmpdir, 1) == 0);
  }

  // Check the expected result.
  filesystem::path expected_prefix(test_tmpdir);
  expected_prefix.append("robotlocomotion_drake_");
  EXPECT_THAT(temp_directory_result,
              ::testing::StartsWith(expected_prefix.string()));
}

GTEST_TEST(TempDirectoryTest, TestTmpdirTrailingSlash) {
  // Temporarily append a '/' to test_tmpdir, and then see what
  // temp_directory() returns.
  const char* test_tmpdir = std::getenv("TEST_TMPDIR");
  ASSERT_STRNE(nullptr, test_tmpdir);
  std::string new_value(test_tmpdir);
  new_value.push_back('/');
  DRAKE_DEMAND(::setenv("TEST_TMPDIR", new_value.c_str(), 1) == 0);
  const std::string temp_directory_result = temp_directory();

  // Revert the environment change.
  DRAKE_DEMAND(::setenv("TEST_TMPDIR", test_tmpdir, 1) == 0);

  // Check the expected result.
  filesystem::path expected_prefix(test_tmpdir);
  expected_prefix.append("robotlocomotion_drake_");
  EXPECT_THAT(temp_directory_result,
              testing::Not(testing::EndsWith("/")));
}

}  // namespace
}  // namespace drake
