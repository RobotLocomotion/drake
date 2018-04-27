#include "drake/common/proto/rpc_pipe_temp_directory.h"

#include <cstdlib>

#include <spruce.hh>

#include "drake/common/drake_throw.h"

namespace drake {
namespace common {

std::string GetRpcPipeTempDirectory() {
  const char* path_str = nullptr;
  (path_str = std::getenv("TEST_TMPDIR")) || (path_str = "/tmp");

  spruce::path path(path_str);
  DRAKE_THROW_UNLESS(path.isDir());

  // Spruce normalizes the path and strips any trailing /.
  return path.getStr();
}

}  // namespace common
}  // namespace drake
