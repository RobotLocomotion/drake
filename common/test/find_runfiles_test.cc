#include "drake/common/find_runfiles.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/filesystem.h"
#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace {

GTEST_TEST(FindRunfilesTest, AcceptanceTest) {
  EXPECT_TRUE(HasRunfiles());
  const auto result = FindRunfile(
      "drake/common/test/find_resource_test_data.txt");
  drake::log()->debug("result.abspath: {}", result.abspath);
  EXPECT_GT(result.abspath.size(), 0);
  EXPECT_TRUE(filesystem::is_regular_file({result.abspath}));
  EXPECT_EQ(result.error, "");
}

GTEST_TEST(FindRunfilesTest, NotDeclaredTest) {
  const auto result = FindRunfile("foo");
  EXPECT_THAT(result.error, testing::MatchesRegex(
      "Sought 'foo' in runfiles directory '.*' but "
      "the file does not exist at that location nor is it on the "
      "manifest; perhaps a 'data = ..' dependency is missing."));
};

GTEST_TEST(FindRunfilesTest, AbsolutePathTest) {
  const auto result = FindRunfile("/dev/null");
  EXPECT_THAT(result.error, testing::MatchesRegex(
      ".*must not be an absolute path.*"));
};

GTEST_TEST(FindRunfilesTest, EmptyPathTest) {
  const auto result = FindRunfile("");
  EXPECT_THAT(result.error, testing::MatchesRegex(
      ".*must not be empty.*"));
};

}  // namespace
}  // namespace drake
