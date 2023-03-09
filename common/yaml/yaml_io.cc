#include "drake/common/yaml/yaml_io.h"

#include <fstream>

namespace drake {
namespace yaml {
namespace internal {

void WriteFile(std::string_view function_name, const std::string& filename,
               const std::string& data) {
  std::ofstream out(filename, std::ios::binary);
  if (out.fail()) {
    throw std::runtime_error(fmt::format("{}() could not open '{}' for writing",
                                         function_name, filename));
  }
  out << data;
  if (out.fail()) {
    throw std::runtime_error(
        fmt::format("{}() could not write to '{}'", function_name, filename));
  }
}

}  // namespace internal
}  // namespace yaml
}  // namespace drake
