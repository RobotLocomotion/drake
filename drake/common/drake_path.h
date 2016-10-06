#pragma once

#include <string>

#include "drake/common/drake_export.h"
#include "drake/common/drake_deprecated.h"

namespace drake {

/// Returns the fully-qualified path to the root of the `drake` source tree.
/// N.B: <em>not</em> the `drake-distro` source tree.
DRAKE_EXPORT std::string GetDrakePath();

/// Legacy compatibility alias for GetDrakePath.
DRAKE_DEPRECATED("Use GetDrakePath instead")
DRAKE_EXPORT std::string getDrakePath();

}  // namespace drake
