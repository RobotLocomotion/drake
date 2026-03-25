#include "drake/common/file_source.h"

#include <fmt/format.h>
#include <fmt/std.h>

namespace drake {

std::string to_string(const FileSource& source) {
  return fmt::to_string(source);
}

}  // namespace drake
