#include "drake/common/yaml/yaml_io_options.h"

namespace drake {
namespace yaml {

std::ostream& operator<<(std::ostream& os, const LoadYamlOptions& x) {
  return os << "{.allow_yaml_with_no_cpp = " << x.allow_yaml_with_no_cpp
            << ", .allow_cpp_with_no_yaml = " << x.allow_cpp_with_no_yaml
            << ", .retain_map_defaults = " << x.retain_map_defaults << "}";
}

}  // namespace yaml
}  // namespace drake
