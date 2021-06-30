#pragma once

// Alias drake::filesystem to either std::filesystem or ghc::filesystem.
//
// The drake::filesystem support is intended ONLY for use within Drake's *.cc
// files -- it is not a public dependency of Drake; do not include this file
// from Drake header files.
//
// Note that until Apple ships a working std::filesystem implementation, we
// need to force-disable it. Similarly, GCC prior to 9 requires arcane linker
// flags, so we need to exclude it as well.
//
// Keep this if-sequence in sync with drake/common/filesystem.cc.
#if __has_include(<filesystem>) && !( \
  defined(__APPLE__) || \
  (!defined(__clang__) && defined(__GNUC__) && (__GNUC__ < 9)))

#include <filesystem>
namespace drake { namespace filesystem = std::filesystem; }

#else

#define GHC_FILESYSTEM_FWD
#include "ghc/filesystem.hpp"
#undef GHC_FILESYSTEM_FWD
namespace drake { namespace filesystem = ghc::filesystem; }

#endif
