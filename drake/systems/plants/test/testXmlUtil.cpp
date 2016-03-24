#include <list>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "drake/systems/plants/xmlUtil.h"

namespace {

TEST(testXmlUtil, testPopulatePackageMap) {
  PackageMap package_map;
  populatePackageMap(package_map);

  std::list<std::string> expected_packages = { "drake", "test" };
  for (auto package: expected_packages) {
    EXPECT_TRUE(package_map.count(package)) <<
        std::string("Expected package not found in list: ")  << package;
  }
}

}
