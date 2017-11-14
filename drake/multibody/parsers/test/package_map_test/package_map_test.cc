#include "drake/multibody/parsers/package_map.h"

#include <algorithm>

#include <gtest/gtest.h>
#include <spruce.hh>

#include "drake/common/find_resource.h"

using std::map;
using std::string;

namespace drake {
namespace parsers {
namespace {

string GetTestDataRoot() {
  const string desired_dir = "drake/multibody/parsers/test/package_map_test/";
  const string contained_file =
      "package_map_test_packages/package_map_test_package_a/package.xml";
  const string absolute_file_path = FindResourceOrThrow(
      desired_dir + contained_file);
  return absolute_file_path.substr(
      0, absolute_file_path.size() - contained_file.size());
}

void VerifyMatch(const PackageMap& package_map,
    const map<string, string>& expected_packages) {
  EXPECT_EQ(package_map.size(), static_cast<int>(expected_packages.size()));
  for (const auto& path_entry : expected_packages) {
    const std::string& package_name = path_entry.first;
    const std::string& package_path = path_entry.second;

    ASSERT_TRUE(package_map.Contains(package_name));
    EXPECT_EQ(package_map.GetPath(package_name), package_path);
  }
}

// Tests that the PackageMap can be manually populated.
GTEST_TEST(PackageMapTest, TestManualPopulation) {
  spruce::dir::mkdir(spruce::path("package_foo"));
  spruce::dir::mkdir(spruce::path("package_bar"));
  map<string, string> expected_packages = {
    {"package_foo", "package_foo"},
    {"my_package", "package_bar"}
  };

  PackageMap package_map;
  for (const auto& it : expected_packages) {
    package_map.Add(it.first, it.second);
  }

  VerifyMatch(package_map, expected_packages);
}

// Tests that PackageMap can be populated by crawling down a directory tree.
GTEST_TEST(PackageMapTest, TestPopulateMapFromFolder) {
  const string root_path = GetTestDataRoot();

  PackageMap package_map;
  package_map.PopulateFromFolder(root_path);

  map<string, string> expected_packages = {
    {"package_map_test_package_a", root_path +
        "package_map_test_packages/package_map_test_package_a/"},
    {"package_map_test_package_b", root_path +
        "package_map_test_packages/package_map_test_package_b/"},
    {"package_map_test_package_c", root_path +
        "package_map_test_packages/package_map_test_package_set/"
        "package_map_test_package_c/"},
    {"package_map_test_package_d", root_path +
        "package_map_test_packages/package_map_test_package_set/"
        "package_map_test_package_d/"}
  };

  VerifyMatch(package_map, expected_packages);
}

// Tests that PackageMap can handle being populated by crawling down a directory
// tree when it is provided a path with extraneous trailing slashes.
GTEST_TEST(PackageMapTest, TestPopulateMapFromFolderExtraTrailingSlashes) {
  const string root_path = GetTestDataRoot();

  PackageMap package_map;
  package_map.PopulateFromFolder(root_path + "///////");

  map<string, string> expected_packages = {
    {"package_map_test_package_a", root_path +
        "package_map_test_packages/package_map_test_package_a/"},
    {"package_map_test_package_b", root_path +
        "package_map_test_packages/package_map_test_package_b/"},
    {"package_map_test_package_c", root_path +
        "package_map_test_packages/package_map_test_package_set/"
        "package_map_test_package_c/"},
    {"package_map_test_package_d", root_path +
        "package_map_test_packages/package_map_test_package_set/"
        "package_map_test_package_d/"}
  };

  VerifyMatch(package_map, expected_packages);
}

// Tests that PackageMap can be populated by crawling up a directory tree.
GTEST_TEST(PackageMapTest, TestPopulateUpstreamToDrake) {
  const string root_path = GetTestDataRoot();
  const string sdf_file_name = FindResourceOrThrow(
      "drake/multibody/parsers/test/package_map_test/"
      "package_map_test_packages/package_map_test_package_a/"
      "sdf/test_model.sdf");

  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(sdf_file_name);

  map<string, string> expected_packages = {
    {"package_map_test_package_a",
        root_path + "package_map_test_packages/package_map_test_package_a"}
  };

  VerifyMatch(package_map, expected_packages);
}

// Tests that PackageMap's streaming to-string operator works.
GTEST_TEST(PackageMapTest, TestStreamingToString) {
  spruce::dir::mkdir(spruce::path("package_foo"));
  spruce::dir::mkdir(spruce::path("package_bar"));
  map<string, string> expected_packages = {
    {"package_foo", "package_foo"},
    {"my_package", "package_bar"}
  };

  PackageMap package_map;
  for (const auto& it : expected_packages) {
    package_map.Add(it.first, it.second);
  }

  std::stringstream string_buffer;
  string_buffer << package_map;
  const std::string resulting_string = string_buffer.str();

  // The following simply tests that the package names and their relative paths
  // exist in the resulting string. It does not check the literal path since
  // that's system dependent or the actual formatting of the text.
  for (const auto& it : expected_packages) {
    EXPECT_NE(resulting_string.find(it.first), std::string::npos);
    EXPECT_NE(resulting_string.find(it.second), std::string::npos);
  }

  // Verifies that there are three lines in the resulting string.
  EXPECT_EQ(std::count(resulting_string.begin(), resulting_string.end(), '\n'),
            3);
}

}  // namespace
}  // namespace parsers
}  // namespace drake
