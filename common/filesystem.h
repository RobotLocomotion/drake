#pragma once

// The drake::filesystem support is intended ONLY for use within Drake's *.cc
// files -- it is not a public dependency of Drake; do not include this file
// from Drake header files.

#include <string>

// Alias drake::filesystem to either std::filesystem or ghc::filesystem.  Note
// that until apple ships a working std::filesystem implementation, we need to
// force-disable it.
// Keep this sequence in sync with drake/common/filesystem.cc.
#if __has_include(<filesystem>) && !defined(__APPLE__)
#include <filesystem>
namespace drake { namespace filesystem = std::filesystem; }
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
#define GHC_FILESYSTEM_FWD
#include "ghc/filesystem.hpp"
#undef GHC_FILESYSTEM_FWD
#pragma GCC diagnostic pop
namespace drake { namespace filesystem = ghc::filesystem; }
#endif

#include "drake/common/drake_deprecated.h"

namespace drake {
namespace internal {

/** Returns true iff the given path is a file. */
DRAKE_DEPRECATED("2019-11-01", "Use drake::filesystem directly.")
inline bool IsFile(const std::string& filesystem_path) {
  return filesystem::is_regular_file({filesystem_path});
}

/** Returns true iff the given path is a directory. */
DRAKE_DEPRECATED("2019-11-01", "Use drake::filesystem directly.")
inline bool IsDir(const std::string& filesystem_path) {
  return filesystem::is_directory({filesystem_path});
}

/** A C++ wrapper for C's readlink(2). */
DRAKE_DEPRECATED("2019-11-01", "Use drake::filesystem directly.")
inline std::string Readlink(const std::string& pathname) {
  return filesystem::read_symlink({pathname}).string();
}

}  // namespace internal
}  // namespace drake
