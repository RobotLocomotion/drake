#include "drake/multibody/parsers/parser_common.h"

#include <string>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"

using std::string;

namespace drake {
namespace parsers {
namespace {

// Verifies that GetFullPath() is able to obtain the full path given a relative
// path to a file in the current working directory.
GTEST_TEST(ParserCommonTest, TestGetFullPath_Relative) {
  const string relative_path = "test_file.txt";
  string full_path;
  ASSERT_NO_THROW(full_path = GetFullPath(relative_path));
  const string expected_full_path = GetDrakePath() +
      "/multibody/parsers/test/parser_common_test/test_file.txt";
  EXPECT_EQ(full_path, expected_full_path);
}

// Verifies that GetFullPath() throws an exception if it is given a relative
// path to a non-existent file in the current working directory.
GTEST_TEST(ParserCommonTest, TestGetFullPathToNonexistentRelativeFile) {
  const string relative_path = "nonexistent_file.txt";
  EXPECT_THROW(GetFullPath(relative_path), std::runtime_error);
}

// Verifies that GetFullPath() is able to obtain the full path given a relative
// path to a file in a subdirectory within the current working directory.
GTEST_TEST(ParserCommonTest, TestGetFullPath_RelativeSubdir) {
  const string relative_path = "subdirectory/nested_file.txt";
  string full_path;
  ASSERT_NO_THROW(full_path = GetFullPath(relative_path));
  const string expected_full_path = GetDrakePath() +
      "/multibody/parsers/test/parser_common_test/subdirectory/nested_file.txt";
  EXPECT_EQ(full_path, expected_full_path);
}

// Verifies that GetFullPath() throws an exception if it is given a relative
// path to a non-existent file in subdirectory within the current working
// directory.
GTEST_TEST(ParserCommonTest, TestGetFullPathToNonexistentRelativeSubdirFile) {
  const string relative_path = "subdirectory/nonexistent_file.txt";
  EXPECT_THROW(GetFullPath(relative_path), std::runtime_error);
}

// Verifies that GetFullPath() throws an exception if it is given a relative
// path of "".
GTEST_TEST(ParserCommonTest, TestGetFullPathToNonexistentFile) {
  EXPECT_THROW(GetFullPath(""), std::runtime_error);
}

// Verifies that GetFullPath() leaves the path alone if it is already a valid
// full path.
GTEST_TEST(ParserCommonTest, TestGetFullPath_Absolute) {
  const string original_path = GetDrakePath() +
      "/multibody/parsers/test/parser_common_test/test_file.txt";
  string full_path;
  full_path = GetFullPath(original_path);
  // ASSERT_NO_THROW(full_path = GetFullPath(original_path));
  EXPECT_EQ(original_path, full_path);
}

// Verifies that GetFullPath() throws an exception if it is given a relative
// path to a non-existent file.
GTEST_TEST(ParserCommonTest, TestGetFullPathToNonexistentAbsoluteFile) {
  const string original_path = GetDrakePath() +
      "/multibody/parsers/test/parser_common_test/nonexistent_file.txt";
  EXPECT_THROW(GetFullPath(original_path), std::runtime_error);
}

}  // namespace
}  // namespace parsers
}  // namespace drake
