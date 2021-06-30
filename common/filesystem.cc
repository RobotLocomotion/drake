// Normally we would #include drake/common/filesystem.h prior to this header
// ("include yourself first"), but we can't do that given the way the ghc
// library implements separate compilation.  So, we have to NOLINT it below.

// Keep this sequence in sync with drake/common/filesystem.h.
#if __has_include(<filesystem>) && !( \
  defined(__APPLE__) || \
  (!defined(__clang__) && defined(__GNUC__) && (__GNUC__ < 9)))

// No compilation required for std::filesystem.

#else

// Compile ghc::filesystem into object code.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated"
#pragma GCC diagnostic ignored "-Wold-style-cast"
#define GHC_FILESYSTEM_IMPLEMENTATION
#include "ghc/filesystem.hpp"  // NOLINT(build/include)
#undef GHC_FILESYSTEM_IMPLEMENTATION
#pragma GCC diagnostic pop

#endif
