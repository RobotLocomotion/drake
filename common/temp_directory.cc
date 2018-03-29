#include "drake/common/temp_directory.h"

#include <cstdlib>

#include <spruce.hh>

#include "drake/common/drake_throw.h"

namespace drake {

// TODO(jamiesnape): This should return a std::filesystem::path as
// std::filesystem::temp_directory_path() does when we can use
// <experimental/filesystem> or C++17 <filesystem>
std::string temp_directory() {
  // Except for querying TEST_TMPDIR, this is the same behavior as
  // std::filesystem::temp_directory_path() in C++17 on POSIX systems.
  const char* path_str = nullptr;
  (path_str = std::getenv("TEST_TMPDIR")) ||
  (path_str = std::getenv("TMPDIR")) ||
  (path_str = std::getenv("TMP")) ||
  (path_str = std::getenv("TEMP")) ||
  (path_str = std::getenv("TEMPDIR")) ||
  (path_str = "/tmp");

  // Spruce normalizes the path and strips any trailing /.
  spruce::path path(path_str);
  DRAKE_THROW_UNLESS(path.isDir());

  return path.getStr();
}

}  // namespace drake
