#pragma once

#include <string>

#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_optional.h"

namespace drake {

/// Returns the fully-qualified path to the root of the `drake` source tree.
DRAKE_DEPRECATED("2019-06-01", "Please use drake::FindResource() instead.")
std::string GetDrakePath();

/// (Advanced) Returns the fully-qualified path to the first folder containing
/// Drake resources as located by FindResource, or nullopt if none is found.
/// For example `${result}/examples/pendulum/Pendulum.urdf` would be the path
/// to the Pendulum example's URDF resource.
///
/// Most users should prefer FindResource() or FindResourceOrThrow() to locate
/// Drake resources for a specific resource filename.  This method only exists
/// for legacy compatibility reasons, and might eventually be removed.
optional<std::string> MaybeGetDrakePath();

}  // namespace drake
