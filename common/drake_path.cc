#include "drake/common/drake_path.h"

#include <spruce.hh>

#include "drake/common/drake_throw.h"
#include "drake/common/find_resource.h"

namespace drake {

std::string GetDrakePath() {
  // Find something that represents where Drake resources live.  This will be
  // "/path/to/drake/.drake-find_resource-sentinel" where "/path/to/" names the
  // root of Drake's git source tree (or perhaps an installed version of same).
  const auto& find_result = FindResource("drake/.drake-find_resource-sentinel");
  spruce::path sentinel_path = find_result.get_absolute_path_or_throw();

  // Take the dirname of sentinel_path; that will name the "drake" folder
  // within the git source tree (or perhaps an installed version of same).
  spruce::path result = sentinel_path.root();
  if (!result.isDir()) {
    throw std::runtime_error("GetDrakePath: proposed result " +
                             result.getStr() + " is not a directory");
  }

  return result.getStr();
}

}  // namespace drake
