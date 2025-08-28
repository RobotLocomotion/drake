#include "drake/common/test_utilities/maybe_pause_for_user.h"

// Manual test users should verify that:
// - the generic prompt string is shown
// - a user-typed RETURN is consumed and the program exits
// - A second prompt string is shown, this time with custom elaboration.
// These results should work for both direct ("bazel-bin/blah") invocation and
// for invocation with "bazel run".

namespace drake {
namespace common {
namespace {

int do_main() {
  MaybePauseForUser();
  MaybePauseForUser("Please ignore this test prompt.");
  return 0;
}

}  // namespace
}  // namespace common
}  // namespace drake

int main() {
  return drake::common::do_main();
}
