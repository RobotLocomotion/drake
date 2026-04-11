#include "drake/common/yaml/yaml_io_options.h"

namespace drake {
namespace yaml {

std::string to_string(const LoadYamlOptions& options) {
  return fmt::format(
      "{{.allow_yaml_with_no_cpp = {}, "
      ".allow_cpp_with_no_yaml = {}, "
      ".retain_map_defaults = {}}}",
      options.allow_yaml_with_no_cpp, options.allow_cpp_with_no_yaml,
      options.retain_map_defaults);
}

std::ostream& operator<<(std::ostream& os, const LoadYamlOptions& x) {
  return os << fmt::to_string(x);
}

}  // namespace yaml
}  // namespace drake
