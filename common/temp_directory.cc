#include "drake/common/temp_directory.h"

#include <cstdlib>

#include <spruce.hh>

#include "drake/common/drake_throw.h"

namespace drake {

std::string temp_directory() {
  // TODO(jamiesnape): Use mkdtemp instead of simply returning /tmp for
  // applications that do not require a hardcoded /tmp.
  const char* path_str = nullptr;
  (path_str = std::getenv("TEST_TMPDIR")) || (path_str = "/tmp");

  // Spruce normalizes the path and strips any trailing /.
  spruce::path path(path_str);
  DRAKE_THROW_UNLESS(path.isDir());

  return path.getStr();
}

}  // namespace drake
