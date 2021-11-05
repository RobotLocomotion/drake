#include "drake/common/yaml/yaml_io.h"

#include <fstream>

namespace drake {
namespace yaml {
namespace internal {

void WriteFile(
    const std::string& filename, const std::string& data) {
  std::ofstream out(filename, std::ios::binary);
  if (out.fail()) {
    throw std::runtime_error(fmt::format(
        "SaveYamlFile() could not open '{}' for writing",
        filename));
  }
  out << data;
  if (out.fail()) {
    throw std::runtime_error(fmt::format(
        "SaveYamlFile() could not write to '{}'",
        filename));
  }
}

}  // namespace internal
}  // namespace yaml
}  // namespace drake
