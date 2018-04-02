#include "drake/common/temp_directory.h"

#include <unistd.h>

#include <cstdlib>
#include <cstring>

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

    char* path_template_str = ::strdup(path_template.getStr().c_str());
    DRAKE_THROW_UNLESS(path_template_str != nullptr);

    path_str = ::mkdtemp(path_template_str);
    std::free(path_template_str);
    DRAKE_THROW_UNLESS(path_str != nullptr);
  }

  // Spruce normalizes the path and strips any trailing /.
  spruce::path path(path_str);
  DRAKE_THROW_UNLESS(path.isDir());

  return path.getStr();
}

}  // namespace drake
