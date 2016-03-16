#include <list>
#include <stdexcept>
#include <string>

#include "drake/systems/plants/xmlUtil.h"

namespace {

void testPopulatePackageMap() {
  PackageMap package_map;
  populatePackageMap(package_map);

  std::list<std::string> expected_packages = { "drake", "test" };
  for (auto package: expected_packages) {
    if (!package_map.count(package)) {
      throw std::runtime_error(
          std::string("Expected package not found in list: ") + package);
    }
  }
}

}

int main(int argc, char* argv[]) {
  testPopulatePackageMap();
}
