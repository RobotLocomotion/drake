#include "drake/common/temp_directory.h"

#include <unistd.h>

#include <cstdlib>

#include "drake/common/drake_throw.h"
#include "drake/common/filesystem.h"

namespace drake {

std::string temp_directory() {
  filesystem::path path;

  const char* tmpdir = nullptr;
  (tmpdir = std::getenv("TEST_TMPDIR")) || (tmpdir = std::getenv("TMPDIR")) ||
      (tmpdir = "/tmp");

  filesystem::path path_template(tmpdir);
  path_template.append("robotlocomotion_drake_XXXXXX");

  std::string path_template_str = path_template.string();
  const char* dtemp = ::mkdtemp(&path_template_str[0]);
  DRAKE_THROW_UNLESS(dtemp != nullptr);

  path = dtemp;

  DRAKE_THROW_UNLESS(filesystem::is_directory(path));
  std::string path_string = path.string();
  DRAKE_DEMAND(path_string.back() != '/');

  return path_string;
}

}  // namespace drake
