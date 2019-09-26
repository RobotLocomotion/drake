// Normally we would #include drake/common/filesystem.h prior to this header
// ("include yourself firsrt"), but we can't do that given the way the ghc
// library implements separate compilation.  So, we have to NOLINT it below.

// Keep this sequence in sync with drake/common/filesystem.h.
#if !__has_include(<filesystem>)
#define GHC_FILESYSTEM_IMPLEMENTATION
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
#include "ghc/filesystem.hpp"  // NOLINT(build/include)
#pragma GCC diagnostic pop
#endif
