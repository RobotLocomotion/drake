#include "drake/common/temp_directory.h"

#include <unistd.h>

#include <cstdlib>

#include <spruce.hh>

#include "drake/common/drake_throw.h"

namespace drake {

std::string temp_directory() {
  spruce::path path;

  const char* test_tmpdir = std::getenv("TEST_TMPDIR");

  if (test_tmpdir == nullptr) {
    const char* tmpdir = nullptr;
    (tmpdir = std::getenv("TMPDIR")) || (tmpdir = "/tmp");

    spruce::path path_template(tmpdir);
    path_template.append("robotlocomotion_drake_XXXXXX");

    std::string path_template_str = path_template.getStr();
    const char* dtemp = ::mkdtemp(&path_template_str[0]);
    DRAKE_THROW_UNLESS(dtemp != nullptr);

    path.setStr(dtemp);
  } else {
    path.setStr(test_tmpdir);
  }

  DRAKE_THROW_UNLESS(path.isDir());

  // Spruce normalizes the path and strips any trailing /.
  return path.getStr();
}

}  // namespace drake
