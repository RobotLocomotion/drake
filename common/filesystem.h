#pragma once

// Alias drake::filesystem to std::filesystem.
//
// The drake::filesystem support is intended ONLY for use within Drake's *.cc
// files -- it is not a public dependency of Drake; do not include this file
// from Drake header files.

#include <filesystem>

namespace drake {

// For legacy reasons, we have a Drake alias for std::filesystem.
// In the future, we could port to `std` directly and remove this alias.
namespace filesystem = std::filesystem;

}  // namespace drake
