#include "drake/common/filesystem.h"

#include <sys/stat.h>
#include <unistd.h>

#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/temp_directory.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace internal {
namespace {

GTEST_TEST(FilesystemTest, ExistTest) {
  EXPECT_FALSE(IsFile("."));
  EXPECT_FALSE(IsFile("common"));
  EXPECT_TRUE(IsFile("common/filesystem_test"));
  EXPECT_FALSE(IsFile("/no/such/path"));

  EXPECT_TRUE(IsDir("."));
  EXPECT_TRUE(IsDir("common"));
  EXPECT_FALSE(IsDir("common/filesystem_test"));
  EXPECT_FALSE(IsDir("/no/such/path"));
}

GTEST_TEST(FilesystemTest, ReadlinkTest) {
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

}  // namespace
}  // namespace internal
}  // namespace drake
