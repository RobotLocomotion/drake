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

// These DRAKE_INTERNAL_VERSION_AT_LEAST_* macros implement the public
// DRAKE_VERSION_AT_LEAST below and are not part of Drake's API. The public
// macro is variadic so that its final `if_unstamped` argument can be optional;
// the chooser selects the 4- or 5-argument implementation by argument count.
#ifndef DRAKE_DOXYGEN_CXX
#define DRAKE_INTERNAL_VERSION_AT_LEAST_4(major, minor, patch, yyyymmdd)    \
  ((DRAKE_INTERNAL_VERSION_MAJOR == 0 && DRAKE_INTERNAL_VERSION_MINOR == 0) \
       ? (DRAKE_INTERNAL_VERSION_PATCH >= (yyyymmdd) && (yyyymmdd) > 0)     \
       : (DRAKE_INTERNAL_VERSION_MAJOR > (major) ||                         \
          (DRAKE_INTERNAL_VERSION_MAJOR == (major) &&                       \
           (DRAKE_INTERNAL_VERSION_MINOR > (minor) ||                       \
            (DRAKE_INTERNAL_VERSION_MINOR == (minor) &&                     \
             DRAKE_INTERNAL_VERSION_PATCH >= (patch))))))
#define DRAKE_INTERNAL_VERSION_AT_LEAST_5(major, minor, patch, yyyymmdd, \
                                          if_unstamped)                  \
  (DRAKE_VERSION_IS_UNSTAMPED                                            \
       ? (if_unstamped)                                                  \
       : DRAKE_INTERNAL_VERSION_AT_LEAST_4(major, minor, patch, yyyymmdd))

// Expands to its sixth argument. DRAKE_VERSION_AT_LEAST passes the caller's
// arguments then the five- and four-argument macros, so `chosen` is the
// five-argument macro for a five-argument call and the four-argument one
// otherwise; its trailing (__VA_ARGS__) then invokes it.
#define DRAKE_INTERNAL_VERSION_AT_LEAST_CHOOSER(arg1, arg2, arg3, arg4, arg5, \
                                                chosen, ...)                  \
  chosen
#endif

/** Evaluates to true iff this build of Drake is at least as new as the given
release. It handles stable releases, nightly/snapshot builds, and unstamped
builds:

- When evaluated against a stable release build, it is true iff the build
  version is greater than or equal to (major, minor, patch); the yyyymmdd
  argument is ignored.
- When evaluated against a nightly or snapshot build, it is true iff the build
  date is at least yyyymmdd and yyyymmdd is nonzero.
- When evaluated against an unstamped build, it is false, unless the optional
  fifth argument if_unstamped is given, in which case that value is used.

The yyyymmdd argument is the nightly date that corresponds to the stable
(major, minor, patch) release. The optional fifth argument, if_unstamped, is
the value to use for an unstamped build (e.g., a plain `bazel build`); it
defaults to false. Prefer true only when you control the build and know it is
recent, because an unstamped build could otherwise be arbitrarily old; see also
`DRAKE_VERSION_IS_UNSTAMPED`.

Intended for use in preprocessor conditionals:

@code{.cpp}
#if DRAKE_VERSION_AT_LEAST(1, 51, 1, 20260311)
// ... use a newer Drake API ...
#endif
@endcode

@hideinitializer */
#define DRAKE_VERSION_AT_LEAST(...)                   \
  DRAKE_INTERNAL_VERSION_AT_LEAST_CHOOSER(            \
      __VA_ARGS__, DRAKE_INTERNAL_VERSION_AT_LEAST_5, \
      DRAKE_INTERNAL_VERSION_AT_LEAST_4)(__VA_ARGS__)

/** Evaluates to true iff this build of Drake is unstamped, i.e., it carries no
version information (`DRAKE_VERSION_STRING` is "unknown"), as happens for a
plain `bazel build` from a source checkout. Stable releases and nightly or
snapshot builds are always stamped, so this is false in those cases.

Because `DRAKE_VERSION_AT_LEAST` by default evaluates to false for an unstamped
build, downstream code can use this macro to detect an unstamped build and
handle it explicitly, e.g., to warn or take a dedicated code path:

@code{.cpp}
#if DRAKE_VERSION_IS_UNSTAMPED
#warning "Drake version is unknown (unstamped build)."
#endif
@endcode

@hideinitializer */
#define DRAKE_VERSION_IS_UNSTAMPED                                           \
  (DRAKE_INTERNAL_VERSION_MAJOR == 0 && DRAKE_INTERNAL_VERSION_MINOR == 0 && \
   DRAKE_INTERNAL_VERSION_PATCH == 0)  // NOLINT(whitespace/indent)

/** @} */
