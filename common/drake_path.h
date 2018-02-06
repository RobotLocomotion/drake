#pragma once

#include <string>

#include "drake/common/drake_deprecated.h"
#include "drake/common/drake_optional.h"

namespace drake {

/// Returns the fully-qualified path to the root of the `drake` source tree.
/// N.B: <em>not</em> the `drake-distro` source tree.
DRAKE_DEPRECATED("Please use drake::FindResource() instead.")
std::string GetDrakePath();

/// Returns the fully-qualified path to the first folder containing Drake
/// resources as located by FindResource, or nullopt if none is found.  For
/// example `${result}/drake/examples/pendulum/Pendulum.urdf` would be the path
/// to the Pendulum example's URDF resource.
optional<std::string> MaybeGetDrakePath();

}  // namespace drake
