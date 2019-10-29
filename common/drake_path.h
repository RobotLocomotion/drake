#pragma once

#include <optional>
#include <string>

#include "drake/common/drake_deprecated.h"

namespace drake {

/// (Advanced) Returns the fully-qualified path to the first folder containing
/// Drake resources as located by FindResource, or nullopt if none is found.
/// For example `${result}/examples/pendulum/Pendulum.urdf` would be the path
/// to the Pendulum example's URDF resource.
///
/// Most users should prefer FindResource() or FindResourceOrThrow() to locate
/// Drake resources for a specific resource filename.  This method only exists
/// for legacy compatibility reasons, and might eventually be removed.
std::optional<std::string> MaybeGetDrakePath();

}  // namespace drake
