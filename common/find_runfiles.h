#pragma once

#include <string>

/** @file
This file contains helpers to work with Bazel-declared runfiles -- declared
data dependencies used by C++ code.  The functions in this file only succeed
when used within a Bazel build.

All source code within Drake should use FindResource() or FindResourceOrThrow()
instead of these Runfiles routines, because FindResource will operate correctly
in pre-compiled / installed builds of Drake whereas FindRunfile will not.

These runfiles-related helpers are intended for use by downstream Bazel
projects that use Drake as a library, so that those projects can reuse the
relatively complicated logic within these routines during source builds.
*/

namespace drake {

/** (Advanced.) Returns true iff this process has Bazel runfiles available.
For both C++ and Python programs, and no matter what workspace a program
resides in (`@drake` or otherwise), this will be true when running
`bazel-bin/pkg/program` or `bazel test //pkg:program` or `bazel run
//pkg:program`. */
bool HasRunfiles();

/** (Advanced.) The return type of FindRunfile(). Exactly one of the two
strings is non-empty. */
struct RlocationOrError {
  /** The absolute path to the resource_path runfile. */
  std::string abspath;
  /** The error message. */
  std::string error;
};

/** (Advanced.) Returns the absolute path to the given resource_path from Bazel
runfiles, or else an error message when not found.  When HasRunfiles() is
false, returns an error. The `resource_path` looks like
`workspace/pkg/subpkg/file.ext`, e.g., "drake/common/foo.txt". */
RlocationOrError FindRunfile(const std::string& resource_path);

}  // namespace drake
