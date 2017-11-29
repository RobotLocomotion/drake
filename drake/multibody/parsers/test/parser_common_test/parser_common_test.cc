#include "drake/multibody/parsers/parser_common.h"

#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <spruce.hh>

using std::string;

namespace drake {
namespace parsers {
namespace {

// Verifies that GetFullPath() promotes a relative path to an absolute path,
// and leaves already-absolute paths alone.
GTEST_TEST(ParserCommonTest, TestGetFullPath_Relative) {
  const string relative_path = "test_file.txt";
  std::ofstream ostr(relative_path);
  ASSERT_TRUE(ostr.is_open());
  ostr.close();

  // Relative path -> absolute path.
  string full_path;
  ASSERT_NO_THROW(full_path = GetFullPath(relative_path));
  ASSERT_TRUE(!full_path.empty());
  EXPECT_EQ(full_path[0], '/');
  spruce::path spruce_full_path(full_path);
  EXPECT_TRUE(spruce_full_path.exists());

  // Absolute path unchanged.
  string full_path_idempotent;
  ASSERT_NO_THROW(full_path_idempotent = GetFullPath(full_path));
  EXPECT_EQ(full_path_idempotent, full_path);
}

// Verifies that GetFullPath() throws when given a relative path to a
// non-existent file.
GTEST_TEST(ParserCommonTest, TestGetFullPathToNonexistentFile) {
  const string relative_path =
      "drake/multibody/parsers/test/parser_common_test/nonexistent_file.txt";
  EXPECT_THROW(GetFullPath(relative_path), std::runtime_error);
}

// Verifies that GetFullPath() throws when given an empty path.
GTEST_TEST(ParserCommonTest, TestGetFullPathOfEmptyPath) {
  EXPECT_THROW(GetFullPath(""), std::runtime_error);
}

}  // namespace
}  // namespace parsers
}  // namespace drake
