#include "drake/common/test_utilities/maybe_pause_for_user.h"

#include <gtest/gtest.h>

// Manual test users should verify that:
// - the program does not pause when invoked with "bazel test".
// - the program does not pause when invoked with "bazel run".
// - Prompt is shown and program pauses for user-typed RETURN when invoked
//   directly ("bazel-bin/blah").

namespace drake {
namespace common {
namespace {

GTEST_TEST(MaybePauseForUserTest, Test) {
  MaybePauseForUser();
  MaybePauseForUser("With message");
}

}  // namespace
}  // namespace common
}  // namespace drake
