#include <list>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "drake/systems/plants/xmlUtil.h"

namespace {

GTEST_TEST(testXmlUtil, testPopulatePackageMap) {
  PackageMap package_map;
  populatePackageMap(package_map);

  std::list<std::string> expected_packages = { "drake", "test" };
  for (auto package : expected_packages) {
    EXPECT_TRUE(package_map.count(package)) <<
        std::string("Expected package not found in list: ")  << package;
  }
}

GTEST_TEST(test_xml_util, test_parse_three_vector_value_1) {
  const char* three_value_string = "1.1 2.2 3.3";
  const char* one_value_string = "4.4";

  Eigen::Vector3d parsed_vector3d;
  Eigen::Vector3d expected_vector3d;
  expected_vector3d << 1.1, 2.2, 3.3;
  EXPECT_TRUE(ParseThreeVectorValue(three_value_string, &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);

  expected_vector3d << 4.4, 4.4, 4.4;
  EXPECT_TRUE(ParseThreeVectorValue(one_value_string, &parsed_vector3d));
  EXPECT_EQ(parsed_vector3d, expected_vector3d);
}

}
