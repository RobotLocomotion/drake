#pragma once

#include <string>

#include "drake/common/drake_deprecated.h"

namespace drake {

/// Returns the fully-qualified path to the root of the `drake` source tree.
/// N.B: <em>not</em> the `drake-distro` source tree.
DRAKE_DEPRECATED("Please use drake::FindResource() instead.")
std::string GetDrakePath();

}  // namespace drake
