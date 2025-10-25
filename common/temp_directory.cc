#include "drake/common/temp_directory.h"

#include <unistd.h>

#include <cstdlib>
#include <filesystem>

#include "drake/common/drake_assert.h"

namespace drake {

namespace fs = std::filesystem;

std::string temp_directory() {
  fs::path path;

  const char* tmpdir = nullptr;
  (tmpdir = std::getenv("TEST_TMPDIR")) || (tmpdir = std::getenv("TMPDIR")) ||
      (tmpdir = "/tmp");

  fs::path path_template(tmpdir);
  path_template.append("robotlocomotion_drake_XXXXXX");

  std::string path_template_str = path_template.string();
  const char* dtemp = ::mkdtemp(&path_template_str[0]);
  DRAKE_THROW_UNLESS(dtemp != nullptr);

  path = dtemp;

  DRAKE_THROW_UNLESS(fs::is_directory(path));
  std::string path_string = path.string();
  DRAKE_DEMAND(path_string.back() != '/');

  return path_string;
}

}  // namespace drake
