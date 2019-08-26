#pragma once

#include <string>

namespace drake {
namespace internal {

/** Returns true iff the given path is a file. */
bool IsFile(const std::string& filesystem_path);

/** Returns true iff the given path is a directory. */
bool IsDir(const std::string& filesystem_path);

/** A C++ wrapper for C's readlink(2). */
std::string Readlink(const std::string& pathname);

}  // namespace internal
}  // namespace drake
