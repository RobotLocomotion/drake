#include "drake/multibody/parsers/package_map.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"

using std::map;
using std::string;

namespace drake {
namespace parsers {
namespace {

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
  map<string, string> expected_packages = {
    {"package_foo", GetDrakePath() + "/multibody/parsers"},
    {"my_package", GetDrakePath() + "/multibody"}
  };

  PackageMap package_map;
  for (auto const& it : expected_packages) {
    package_map.Add(it.first, it.second);
  }

  VerifyMatch(package_map, expected_packages);
}

// Tests that PackageMap can be populated by crawling down a directory tree.
GTEST_TEST(PackageMapTest, TestPopulateMapFromFolder) {
  const string root_path(GetDrakePath() +
                         "/multibody/parsers/test/package_map_test/");

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
  const string root_path(GetDrakePath() +
                         "/multibody/parsers/test/package_map_test/");

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
  const string sdf_file_name(GetDrakePath() +
      "/multibody/parsers/test/package_map_test/package_map_test_packages/"
      "package_map_test_package_a/sdf/test_model.sdf");

  PackageMap package_map;
  package_map.PopulateUpstreamToDrake(sdf_file_name);

  map<string, string> expected_packages = {
    {"package_map_test_package_a",
        GetDrakePath() + "/multibody/parsers/test/package_map_test/"
                         "package_map_test_packages/package_map_test_package_a"}
  };

  VerifyMatch(package_map, expected_packages);
}

}  // namespace
}  // namespace parsers
}  // namespace drake
