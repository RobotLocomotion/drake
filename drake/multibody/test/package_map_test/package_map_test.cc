#include "drake/multibody/package_map.h"

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"

using std::map;
using std::string;

namespace drake {
namespace parsers {
namespace {

void VerifyMatch(const PackageMap& package_map,
    const map<string, string>& expected_packages) {
  for (const auto& path_entry : expected_packages) {
    const std::string& package_name = path_entry.first;
    const std::string& package_path = path_entry.second;

    ASSERT_TRUE(package_map.Contains(package_name));
    EXPECT_EQ(package_map.GetPath(package_name), package_path);
  }
}

GTEST_TEST(PackageMapTest, TestManualPopulation) {
  PackageMap package_map;
  package_map.Add("package_foo", "foo/bar/baz");
  package_map.Add("my_package", "my/mysterious/package");

  map<string, string> expected_packages = {
    {"package_foo", "foo/bar/baz"},
    {"my_package", "my_mysterious/package"}
  };

  VerifyMatch(package_map, expected_packages);
}

GTEST_TEST(PackageMapTest, TestPopulateMapFromFolder) {
  const string root_path(GetDrakePath() + "/multibody/test/package_map_test/");

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

}  // namespace
}  // namespace parsers
}  // namespace drake
