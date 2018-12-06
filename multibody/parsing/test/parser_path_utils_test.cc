#include "drake/multibody/parsing/parser_path_utils.h"

#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <spruce.hh>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"

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

  // file scheme requires an absolute path.
  const std::string absolute_path = "/drake/multibody/parsing/test/"
      "package_map_test_packages/package_map_test_package_a/sdf/test_model.sdf";

  // Set the root directory - it will not end up being used in ResolveUri().
  const std::string root_dir = ".";

  const auto uri = std::string("file://") + absolute_path;
  string path = ResolveUri(uri, package_map, root_dir);
  EXPECT_EQ(path, absolute_path);
}

// Verifies that ResolveUri() resolves to the proper file using no scheme and
// an absolute path.
GTEST_TEST(ResolveUriTest, TestAbsolutePath) {
  // Use an empty package map.
  PackageMap package_map;

  const std::string absolute_path = "/dev/null";

  // Set the root directory.
  const std::string root_dir = "/";

  // We can't test for the exact path since it will change on every system.
  string path = ResolveUri(absolute_path, package_map, root_dir);
  EXPECT_EQ(path, absolute_path);
}

// Verifies that ResolveUri() resolves to the proper file using no scheme and
// a relative path.
GTEST_TEST(ResolveUriTest, TestRelativePath) {
  // Use an empty package map.
  PackageMap package_map;

  const std::string relative_path = "package_map_test_packages/"
      "package_map_test_package_a/sdf/test_model.sdf";

  // Set the root directory.
  const std::string root_dir = "multibody/parsing/test/";

  // We can't test for the exact path since it will change on every system.
  string path = ResolveUri(relative_path, package_map, root_dir);
  EXPECT_NE(path, "");
}

// Verifies that ResolveUri() resolves to the proper file using the scheme
// 'model://'
GTEST_TEST(ResolveUriTest, TestModel) {
  const string sdf_file_name = FindResourceOrThrow(
      "drake/multibody/parsing/test/"
          "package_map_test_packages/package_map_test_package_a/"
          "sdf/test_model.sdf");

  // Create the package map.
  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(sdf_file_name);

  // Set the root directory - it will not end up being used in ResolveUri().
  const std::string root_dir = "drake/multibody/parsing/test/";

  // Create the URI.
  const string uri = "model://package_map_test_package_a/sdf/test_model.sdf";

  string path = ResolveUri(uri, package_map, root_dir);
  EXPECT_EQ(path, sdf_file_name);
}

// Verifies that ResolveUri() chokes on an unsupported scheme (like http://)
GTEST_TEST(ResolveUriTest, TestUnsupported) {
  PackageMap package_map;
  const std::string root_dir = ".";
  const string uri = "http://localhost";
  DRAKE_EXPECT_THROWS_MESSAGE(
      ResolveUri(uri, package_map, root_dir),
      std::runtime_error, "URI specifies an unsupported scheme. Supported "
          "schemes are 'file://', 'model://', and 'package://'. Provided URI:"
          ".*");
}

}  // namespace
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
