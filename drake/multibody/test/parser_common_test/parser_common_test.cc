#include "drake/multibody/parser_urdf.h"

#include <iostream>
#include <fstream>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/rigid_body_tree.h"

using std::endl;
using std::ifstream;
using std::make_unique;
using std::string;
using std::stringstream;
using std::unique_ptr;

namespace drake {

using multibody::joints::kQuaternion;
using parsers::urdf::AddModelInstanceFromUrdfFileWithRpyJointToWorld;
using parsers::urdf::AddModelInstanceFromUrdfStringWithRpyJointToWorld;
using parsers::urdf::AddModelInstanceFromUrdfString;

namespace parsers {
namespace {

const std::string kRootPath(GetDrakePath() +
    "/multibody/test/parser_common_test/");

GTEST_TEST(ParserCommonTest, TestPopulateMapFromFolder) {
  PackageMap package_map;
  PopulateMapFromFolder(kRootPath, &package_map);

  PackageMap expected_package_map = {
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

  EXPECT_EQ(package_map, expected_package_map);
}

}  // namespace
}  // namespace parsers
}  // namespace drake
