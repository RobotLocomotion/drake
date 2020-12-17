/* clang-format off to disable clang-format-includes */
#include "drake/common/find_runfiles.h"
/* clang-format on */

// This is a stubbed-out implementation of find_runfiles.h.  Its purpose is for
// downstream users who choose not to run Drake's CMakeLists.txt nor BUILD
// files, but instead are writing their own build system for a subset of Drake.
//
// By providing this stub, those users can avoid the need to satisfy the
// include statement for "tools/cpp/runfiles/runfiles.h" when compiling
// find_runfiles.cc -- that header is provided internally by Bazel.
//
// This implementation is NOT used by the Bazel build (outside of the unit
// test); it's not even used in the Bazel-installed binary release build (e.g.,
// as called by Drake's CMakeLists.txt files).  It is nominally dead code.

namespace drake {

bool HasRunfiles() {
  return false;
}

RlocationOrError FindRunfile(const std::string& resource_path) {
  RlocationOrError result;
  result.error = "FindRunfile is stubbed out";
  return result;
}

}  // namespace drake
