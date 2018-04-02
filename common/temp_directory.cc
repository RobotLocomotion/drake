#include "drake/common/temp_directory.h"

#include <unistd.h>

#include <cstdlib>

#include <spruce.hh>

#include "drake/common/drake_throw.h"

namespace drake {

std::string temp_directory() {
  const char* path_str = std::getenv("TEST_TMPDIR");

  if (path_str == nullptr) {
      char path_template[] = "/tmp/robotlocomotion_drake_XXXXXX";
      path_str = ::mkdtemp(path_template);
      DRAKE_THROW_UNLESS(path_str != nullptr);
  }

  // Spruce normalizes the path and strips any trailing /.
  spruce::path path(path_str);
  DRAKE_THROW_UNLESS(path.isDir());

  return path.getStr();
}

}  // namespace drake
