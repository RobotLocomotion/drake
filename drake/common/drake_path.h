#pragma once

#include <string>

#include "drake/common/drake_compat.h"

namespace drake {

/// Returns the fully-qualified path to the root of the `drake` source tree.
/// N.B: <em>not</em> the `drake-distro` source tree.
std::string GetDrakePath();

}  // namespace drake
