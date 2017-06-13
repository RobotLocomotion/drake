// This file is the Bazel build's implementation of drake_path.h.
#include "drake/common/drake_path.h"

#include <spruce.hh>

#include "drake/common/drake_throw.h"
#include "drake/common/find_resource.h"

namespace drake {

std::string GetDrakePath() {
  // Find something that represents where Drake resources live.  This will be
  // "/path/to/.drake-resource-sentinel" where "/path/to/" names the root of
  // Drake's git source tree (or perhaps an installed version of the same).
  const auto& find_result = FindResource(".drake-resource-sentinel");
  spruce::path sentinel_path = find_result.get_absolute_path_or_throw();

  // Rewrite to be the "/path/to/drake" that names the "drake" folder within
  // the git source tree (or perhaps an installed version of the same).
  spruce::path result = sentinel_path.root();
  result.append("drake");
  if (!result.isDir()) {
    throw std::runtime_error("GetDrakePath: proposed result " +
                             result.getStr() + " is not a directory");
  }

  return result.getStr();
}

}  // namespace drake
