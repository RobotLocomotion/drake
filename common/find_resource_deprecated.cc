#include <cstdlib>
#include <stdexcept>

#include <fmt/format.h>

#include "drake/common/drake_throw.h"
#include "drake/common/find_resource.h"
#include "drake/common/never_destroyed.h"

// This file contains the implementations of find_resource.h functions that are
// deprecated, in order to de-clutter find_resource.cc.

namespace drake {
namespace {
std::vector<std::string>& GetMutableResourceSearchPaths() {
  static never_destroyed<std::vector<std::string>> search_paths;
  return search_paths.access();
}
}  // namespace

std::vector<std::string> GetResourceSearchPaths() {
  return GetMutableResourceSearchPaths();
}

void AddResourceSearchPath(const std::string& search_path) {
  DRAKE_THROW_UNLESS((search_path.length() > 0) && (search_path[0] == '/'));

  // When the environment variable is aready in play, this function has no
  // effect.
  if (::getenv(kDrakeResourceRootEnvironmentVariableName)) {
    throw std::runtime_error(fmt::format(
        "Cannot AddResourceSearchPath when {} is already set.",
        kDrakeResourceRootEnvironmentVariableName));
  }

  // Set the environment variable so that `search_path` will be considered.
  ::setenv(kDrakeResourceRootEnvironmentVariableName, search_path.c_str(), 1);

  // Add it to the collection so that GetResourceSearchPaths still works.
  GetMutableResourceSearchPaths().push_back(std::move(search_path));
}

}  // namespace drake
