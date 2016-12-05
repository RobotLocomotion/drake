#include "drake/examples/examples_package_map.h"

#include <string>

#include "drake/common/drake_path.h"

using std::string;

namespace drake {

using parsers::PackageMap;

namespace examples {
namespace {

// Adds key @p name and value @p path to @p package_map.
// Aborts if @p package_map is nullptr or @p name is already a key in
// @p package_map.
void AddPackage(const string& name, const string& path,
    PackageMap* package_map) {
  DRAKE_DEMAND(package_map != nullptr);
  DRAKE_DEMAND(package_map->find(name) == package_map->end());
  package_map->insert(std::make_pair(name, path));
}

}  // namespace

void AddExamplePackages(PackageMap* package_map) {
  AddPackage("Atlas", GetDrakePath() + "/examples/Atlas/", package_map);
  AddPackage("IRB140", GetDrakePath() + "/examples/IRB140/", package_map);
  AddPackage("Valkyrie", GetDrakePath() + "/examples/Valkyrie", package_map);
  AddPackage("hsrb_description", GetDrakePath() + "/../ros/hsrb_description",
      package_map);
}

}  // namespace examples
}  // namespace drake
