#include "drake/multibody/parsing/parser_path_utils.h"

#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <spruce.hh>

using std::string;

namespace drake {
namespace multibody {
namespace parsing {
namespace {

// Verifies that GetFullPath() promotes a relative path to an absolute path,
// and leaves already-absolute paths alone.
GTEST_TEST(ParserPathUtilsTest, TestGetFullPath_Relative) {
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
GTEST_TEST(ParserPathUtilsTest, TestGetFullPathToNonexistentFile) {
  const string relative_path =
      "drake/multibody/parsers/test/parser_common_test/nonexistent_file.txt";
  EXPECT_THROW(GetFullPath(relative_path), std::runtime_error);
}

// Verifies that GetFullPath() throws when given an empty path.
GTEST_TEST(ParserPathUtilsTest, TestGetFullPathOfEmptyPath) {
  EXPECT_THROW(GetFullPath(""), std::runtime_error);
}

// Verifies that ResolveUri() resolves the proper file using the scheme
// 'file://'
GTEST_TEST(ResolveUriTest, TestFile) {
  // Use an empty package map.
  PackageMap package_map;

  // Set the root directory.
  const std::string root_dir = "drake/multibody/parsing/test/";
  const std::string relative_path = "package_map_test_packages/"
      "package_map_test_package_a/sdf/test_model.sdf";

  const auto uri = std::string("file://") + relative_path;
  string path = ResolveUri(uri, package_map, root_dir);
  EXPECT_EQ(path, root_dir + relative_path);
}

}  // namespace
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
