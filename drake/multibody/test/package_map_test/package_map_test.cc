#include "drake/multibody/package_map.h"

using std::map;
using std::string;

namespace drake {
namespace parsers {
namespace {

const std::string kRootPath(GetDrakePath() +
	                        "/multibody/test/package_map_test/");

GTEST_TEST(PackageMapTest, TestManualPopulation) {
  // TODO
}

GTEST_TEST(PackageMapTest, TestPopulateMapFromFolder) {
  PackageMap package_map;
  package_map.PopulateFromFolder(kRootPath);

  map<string, string> expected_packages = {
    {"package_map_test_package_a", kRootPath +
        "package_map_test_packages/package_map_test_package_a/"},
    {"package_map_test_package_b", kRootPath +
        "package_map_test_packages/package_map_test_package_b/"},
    {"package_map_test_package_c", kRootPath +
        "package_map_test_packages/package_map_test_package_set/"
        "package_map_test_package_c/"},
    {"package_map_test_package_d", kRootPath +
        "package_map_test_packages/package_map_test_package_set/"
        "package_map_test_package_d/"}};

  for (const auto& path_entry : expected_packages) {
    const std::string& package_name = path_entry.first;
    const std::string& package_path = path_entry.second;

    ASSERT_TRUE(package_map.Contains(package_name));
    EXPECT_EQ(package_map.GetPath(package_name), package_name);
  }
}

}  // namespace
}  // namespace parsers
}  // namespace drake
