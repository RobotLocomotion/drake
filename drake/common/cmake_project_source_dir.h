#pragma once

#include <string>

namespace drake {
namespace detail {

/// Returns the fully-qualified path to the root of the `drake` source tree.
/// N.B: <em>not</em> the `drake-distro` source tree.
std::string GetCMakeProjectSourceDir();

}  // namespace detail
}  // namespace drake
