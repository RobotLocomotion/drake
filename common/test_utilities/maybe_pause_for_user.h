#pragma once

namespace drake {
namespace common {

/** Pauses execution for the user to examine visualization output, for use
 within bazel unit tests. This function will not pause execution when running
 as a bazel test, but when running as a command-line executable, it will enable
 users to see the visualization outputs.

 The complete behavior is somewhat more subtle, depending on the bazel rules
 used to build the program, and the way it is invoked:

 - built with a bazel test rule
   - invoked with "bazel test" -- does not pause
   - invoked with "bazel run" -- does not pause
   - invoked directly ("bazel/bin/[PROGRAM]") -- does pause
 - built with a bazel binary rule
   - invoked with "bazel run" -- does pause
   - invoked directly ("bazel/bin/[PROGRAM]") -- does pause
*/
void MaybePauseForUser();

}  // namespace common
}  // namespace drake


