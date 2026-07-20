#include "drake/version.h"

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

GTEST_TEST(VersionTest, AtLeastIsSelfConsistent) {
  // Every build is at least as new as itself, but not as new as itself plus
  // one. These hold whether we are a stable release or a nightly/dev build (in
  // the latter regime the yyyymmdd argument does the work, in the former the
  // major/minor/patch triple does).
  EXPECT_TRUE(DRAKE_VERSION_AT_LEAST(  //
      DRAKE_VERSION_MAJOR, DRAKE_VERSION_MINOR, DRAKE_VERSION_PATCH,
      DRAKE_VERSION_PATCH));
  EXPECT_FALSE(DRAKE_VERSION_AT_LEAST(  //
      DRAKE_VERSION_MAJOR, DRAKE_VERSION_MINOR, DRAKE_VERSION_PATCH + 1,
      DRAKE_VERSION_PATCH + 1));

  // The zero release is the floor: every build is at least (0, 0, 0, 0).
  EXPECT_TRUE(DRAKE_VERSION_AT_LEAST(0, 0, 0, 0));
}

}  // namespace
}  // namespace drake
