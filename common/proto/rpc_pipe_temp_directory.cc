#include "drake/common/proto/rpc_pipe_temp_directory.h"

#include <cstdlib>
#include <filesystem>

#include "drake/common/drake_assert.h"

namespace drake {
namespace common {

std::string GetRpcPipeTempDirectory() {
  const char* path_str = nullptr;
  (path_str = std::getenv("TEST_TMPDIR")) || (path_str = "/tmp");

  const std::filesystem::path path(path_str);
  DRAKE_THROW_UNLESS(std::filesystem::is_directory(path));
  return path.string();
}

}  // namespace common
}  // namespace drake
