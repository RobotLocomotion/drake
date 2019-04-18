#pragma once

#include <string>

namespace drake {
namespace internal {

/** Returns true iff this process has Bazel runfiles available.  For both C++
and Python programs, and no matter what workspace a program resides in (`@drake`
or otherwise), this will be true when running `bazel-bin/pkg/program` or `bazel
test //pkg:program` or `bazel run //pkg:program`. */
bool HasRunfiles();

/** The return type of FindRunfile(). Exactly one of the two strings is
non-empty. */
struct RlocationOrError {
  /** The absolute path to the resource_path runfile. */
  std::string abspath;
  /** The error message. */
  std::string error;
};

/** Returns the absolute path to the given resource_path from Bazel runfiles,
or else an error message when not found.  When HasRunfiles() is false, returns
an error. */
RlocationOrError FindRunfile(const std::string& resource_path);

/** Returns true iff the given path is a file. */
bool IsFile(const std::string& filesystem_path);

/** Returns true iff the given path is a directory. */
bool IsDir(const std::string& filesystem_path);

/** A C++ wrapper for C's readlink(2). */
std::string Readlink(const std::string& pathname);

}  // namespace internal
}  // namespace drake
