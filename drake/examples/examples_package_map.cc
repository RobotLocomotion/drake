#include "drake/examples/examples_package_map.h"

#include <string>

#include "drake/common/drake_path.h"

using std::string;

namespace drake {

using parsers::AddPackage;
using parsers::PackageMap;

namespace examples {

// TODO(liang.fok): Remove the reliance on GetDrakePath(). Usage of
// GetDrakePath() is not good because it hard-codes a path that is determined at
// compile time. This will not work once Drake has binary releases since the
// path to the models will differ on the machine on which the binaries are run.
// See: #1471.
void AddExamplePackages(PackageMap* package_map) {
  AddPackage("Atlas", GetDrakePath() + "/examples/Atlas/", package_map);
  AddPackage("IRB140", GetDrakePath() + "/examples/IRB140/", package_map);
  AddPackage("Valkyrie", GetDrakePath() + "/examples/Valkyrie", package_map);
  AddPackage("hsrb_description", GetDrakePath() + "/../ros/hsrb_description",
      package_map);
}

}  // namespace examples
}  // namespace drake
