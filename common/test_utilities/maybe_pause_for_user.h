#pragma once

namespace drake {
namespace common {

/** Pauses execution for the user to examine visualization output, for use
 within bazel unit tests. This function will not pause execution when running
 as a bazel test, but when running as a command-line executable, it will enable
 users to see the visualization outputs. */
void MaybePauseForUser();

}  // namespace common
}  // namespace drake


