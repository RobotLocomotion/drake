#include "drake/common/temp_directory.h"

#include <unistd.h>

#include <cstdlib>

#include <spruce.hh>

#include "drake/common/drake_throw.h"

namespace drake {

std::string temp_directory() {
  const char* path_str = std::getenv("TEST_TMPDIR");

  if (path_str == nullptr) {
    const char* tmpdir_str = nullptr;
    (tmpdir_str = std::getenv("TMPDIR")) || (tmpdir_str = "/tmp");

    spruce::path path_template(tmpdir_str);
    path_template.append("robotlocomotion_drake_XXXXXX");

    path_str = ::mkdtemp(&path_template.getStr()[0]);
    DRAKE_THROW_UNLESS(path_str != nullptr);
  }

  // Spruce normalizes the path and strips any trailing /.
  spruce::path path(path_str);
  DRAKE_THROW_UNLESS(path.isDir());

  return path.getStr();
}

}  // namespace drake
