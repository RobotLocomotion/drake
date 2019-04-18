#include "drake/common/find_runfiles.h"

#include <sys/stat.h>
#include <unistd.h>

#include <fstream>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace internal {
namespace {

GTEST_TEST(FindRunfilesTest, ExistTest) {
  EXPECT_FALSE(IsFile("."));
  EXPECT_FALSE(IsFile("common"));
  EXPECT_TRUE(IsFile("common/find_runfiles_test"));
  EXPECT_FALSE(IsFile("/no/such/path"));

  EXPECT_TRUE(IsDir("."));
  EXPECT_TRUE(IsDir("common"));
  EXPECT_FALSE(IsDir("common/find_runfiles_test"));
  EXPECT_FALSE(IsDir("/no/such/path"));
}

GTEST_TEST(FindRunfilesTest, ReadlinkTest) {
  std::string tmp = temp_directory();
  std::string rel_file = tmp + "/rel_file";
  std::string rel_dir = tmp + "/rel_dir";
  ASSERT_EQ(::symlink("a_file", rel_file.c_str()), 0);
  ASSERT_EQ(::symlink("a_dir", rel_dir.c_str()), 0);
  EXPECT_EQ(Readlink(rel_file), "a_file");
  EXPECT_EQ(Readlink(rel_dir), "a_dir");

  EXPECT_FALSE(IsFile(rel_file));
  EXPECT_FALSE(IsDir(rel_dir));
  EXPECT_FALSE(IsFile(rel_dir));
  EXPECT_FALSE(IsDir(rel_file));

  ASSERT_EQ(::mkdir((tmp + "/a_dir").c_str(), S_IRWXU), 0);
  std::ofstream(tmp + "/a_file", std::ios::out).close();

  EXPECT_TRUE(IsFile(rel_file));
  EXPECT_TRUE(IsDir(rel_dir));
  EXPECT_FALSE(IsFile(rel_dir));
  EXPECT_FALSE(IsDir(rel_file));

  DRAKE_EXPECT_THROWS_MESSAGE(
      Readlink("/no_such_readlink"), std::exception,
      "Could not open /no_such_readlink");
}

GTEST_TEST(FindRunfilesTest, AcceptanceTest) {
  EXPECT_TRUE(HasRunfiles());
  const auto result = FindRunfile(
      "drake/common/test/find_resource_test_data.txt");
  EXPECT_GT(result.abspath.size(), 0);
  EXPECT_TRUE(IsFile(result.abspath));
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
}  // namespace internal
}  // namespace drake
