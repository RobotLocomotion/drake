#include "drake/common/drake_path.h"

#include <spruce.hh>

#include "drake/common/find_resource.h"

// N.B. This code is unit tested in test/find_resource_test.cc.

namespace drake {
namespace {

optional<std::string> MaybeGetDrakePathImpl(bool should_throw) {
  // Find something that represents where Drake resources live.  This will be
  // "/path/to/drake/.drake-find_resource-sentinel" where "/path/to/" names the
  // root of Drake's git source tree (or perhaps an installed version of same).
  const auto& find_result = FindResource("drake/.drake-find_resource-sentinel");
  if (find_result.get_error_message() && !should_throw) {
    return nullopt;
  }
  spruce::path sentinel_path = find_result.get_absolute_path_or_throw();

  // Take the dirname of sentinel_path, so that we have a folder where looking
  // up "drake/foo/bar.urdf" makes sense.
  return sentinel_path.root();
}

}  // namespace

optional<std::string> MaybeGetDrakePath() {
  return MaybeGetDrakePathImpl(false);
}

std::string GetDrakePath() {
  return *MaybeGetDrakePathImpl(true);
}

}  // namespace drake
