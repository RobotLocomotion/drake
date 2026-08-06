#pragma once

// This header provides preprocessor macros for reporting and comparing the
// Drake version, derived from the same version stamp that populates
// drake-config.cmake (see drake/tools/install/libdrake). For an unstamped
// build (e.g., a plain `bazel build`), the version string is "unknown".

#ifndef DRAKE_DOXYGEN_CXX
#include "drake/version_internal.h"
#endif

/** @defgroup drake_versioning Drake Version
@ingroup technical_notes
@{

Drake exposes its version to downstream C++ as preprocessor macros, so that
code can adapt to the Drake version at compile time, e.g., to handle API
changes across releases.

Python users should instead obtain the version at runtime via
`importlib.metadata.version("%drake")`. */

/** Drake's full version string. For a versioned release this looks like
"1.51.1"; for a nightly or snapshot build like "0.0.20260721.143022+gitabc";
for an unstamped build it is "unknown".
@hideinitializer */
#define DRAKE_VERSION_STRING DRAKE_INTERNAL_VERSION_STRING

/** Evaluates to true iff this build of Drake is at least as new as the given
release. It handles stable releases, nightly/snapshot builds, and unstamped
builds:

- When evaluated against a stable release build, it is true iff the build
  version is greater than or equal to (major, minor, patch); the yyyymmdd
  argument is ignored.
- When evaluated against a nightly or snapshot build, it is true iff the build
  date is at least yyyymmdd and yyyymmdd is nonzero.
- When evaluated against an unstamped build, it is always false.

The yyyymmdd argument is the nightly date that corresponds to the stable
(major, minor, patch) release. Intended for use in preprocessor conditionals:

@code{.cpp}
#if DRAKE_VERSION_AT_LEAST(1, 51, 1, 20260311)
// ... use a newer Drake API ...
#endif
@endcode

@hideinitializer */
#define DRAKE_VERSION_AT_LEAST(major, minor, patch, yyyymmdd)               \
  ((DRAKE_INTERNAL_VERSION_MAJOR == 0 && DRAKE_INTERNAL_VERSION_MINOR == 0) \
       ? (DRAKE_INTERNAL_VERSION_PATCH >= (yyyymmdd) && (yyyymmdd) > 0)     \
       : (DRAKE_INTERNAL_VERSION_MAJOR > (major) ||                         \
          (DRAKE_INTERNAL_VERSION_MAJOR == (major) &&                       \
           (DRAKE_INTERNAL_VERSION_MINOR > (minor) ||                       \
            (DRAKE_INTERNAL_VERSION_MINOR == (minor) &&                     \
             DRAKE_INTERNAL_VERSION_PATCH >= (patch))))))

/** @} */
