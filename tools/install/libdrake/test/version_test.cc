#include "drake/version.h"

#include <regex>
#include <string>

#include <gtest/gtest.h>

namespace drake {
namespace {

// The numeric components must be usable as integer constant expressions (i.e.,
// they are real integers, not the string "unknown").
static_assert(DRAKE_VERSION_MAJOR >= 0);
static_assert(DRAKE_VERSION_MINOR >= 0);
static_assert(DRAKE_VERSION_PATCH >= 0);
static_assert(DRAKE_VERSION_TWEAK >= 0);

// The version string is always a non-empty string literal.
static_assert(sizeof(DRAKE_VERSION_STR) > 1);

// Exercises DRAKE_VERSION_AT_LEAST against whatever version this build reports,
// without assuming which kind of build it is (stable release, nightly, or an
// unstamped "unknown" build).
GTEST_TEST(VersionTest, Basic) {
  // No build is ever at least as new as an absurd future release-and-date.
  EXPECT_FALSE(DRAKE_VERSION_AT_LEAST(9999, 0, 0, 99999999));

  if (DRAKE_VERSION_MAJOR != 0 || DRAKE_VERSION_MINOR != 0) {
    // Stable release: at least as new as itself, but not one patch newer. The
    // yyyymmdd argument is ignored on this branch.
    EXPECT_TRUE(DRAKE_VERSION_AT_LEAST(DRAKE_VERSION_MAJOR, DRAKE_VERSION_MINOR,
                                       DRAKE_VERSION_PATCH, 0));
    EXPECT_FALSE(DRAKE_VERSION_AT_LEAST(
        DRAKE_VERSION_MAJOR, DRAKE_VERSION_MINOR, DRAKE_VERSION_PATCH + 1, 0));
  } else if (DRAKE_VERSION_PATCH != 0) {
    // Nightly/snapshot: at least as new as its own build date, but not a later
    // one.
    EXPECT_TRUE(DRAKE_VERSION_AT_LEAST(0, 0, 0, DRAKE_VERSION_PATCH));
    EXPECT_FALSE(DRAKE_VERSION_AT_LEAST(0, 0, 0, DRAKE_VERSION_PATCH + 1));
    EXPECT_FALSE(DRAKE_VERSION_AT_LEAST(9999, 0, 0, 0));
  } else {
    // Unstamped ("unknown") build: never at least as new as any real release.
    EXPECT_FALSE(DRAKE_VERSION_AT_LEAST(0, 0, 0, 0));
    EXPECT_FALSE(DRAKE_VERSION_AT_LEAST(1, 51, 1, 20260311));
  }
}

GTEST_TEST(VersionTest, ComponentsFormDottedQuad) {
  // The four numeric components always form a "major.minor.patch.tweak" quad,
  // e.g. "1.51.1.0" or "0.0.0.0", whatever the build.
  const std::string version = std::to_string(DRAKE_VERSION_MAJOR) + "." +
                              std::to_string(DRAKE_VERSION_MINOR) + "." +
                              std::to_string(DRAKE_VERSION_PATCH) + "." +
                              std::to_string(DRAKE_VERSION_TWEAK);
  EXPECT_TRUE(std::regex_match(version, std::regex(R"(\d+\.\d+\.\d+\.\d+)")))
      << "version components do not form a numeric quad: " << version;
}

}  // namespace
}  // namespace drake
