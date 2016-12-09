#include "drake/multibody/package_map.h"

using std::map;
using std::string;

namespace drake {
namespace parsers {
namespace {

const std::string kRootPath(GetDrakePath() +
	                        "/multibody/test/package_map_test/");

GTEST_TEST(PackageMapTest, TestPopulateMapFromFolder) {
  PackageMap package_map;
  package_map.PopulateFromFolder(kRootPath);

  map<string, string> expected_package_map = {
    {"parser_common_test_package_a", kRootPath +
        "parser_common_test_packages/parser_common_test_package_a/"},
    {"parser_common_test_package_b", kRootPath +
        "parser_common_test_packages/parser_common_test_package_b/"},
    {"parser_common_test_package_c", kRootPath +
        "parser_common_test_packages/parser_common_test_package_set/"
        "parser_common_test_package_c/"},
    {"parser_common_test_package_d", kRootPath +
        "parser_common_test_packages/parser_common_test_package_set/"
        "parser_common_test_package_d/"}};

  // TODO ...finish
}

}  // namespace
}  // namespace parsers
}  // namespace drake
