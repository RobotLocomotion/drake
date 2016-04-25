#pragma once

#include <string>

#include "drake/drakeCommon_export.h"

namespace drake {

/// Returns the fully-qualified path to the root of the `drake` source tree.
/// N.B: <em>not</em> the `drake-distro` source tree.
DRAKECOMMON_EXPORT
std::string getDrakePath();

}  // namespace drake
