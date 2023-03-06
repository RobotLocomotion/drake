#pragma once

#include <ostream>

#include "drake/common/fmt_ostream.h"

namespace drake {
namespace yaml {

/** Configuration for LoadYamlFile() and LoadYamlString() to govern when certain
conditions are errors or not. Refer to the member fields for details. */
struct LoadYamlOptions {
  friend std::ostream& operator<<(std::ostream&, const LoadYamlOptions&);

  /** Allows yaml Maps to have extra key-value pairs that are not Visited by the
  Serializable being parsed into. In other words, the Serializable types provide
  an incomplete schema for the YAML data. This allows for parsing only a subset
  of the YAML data. */
  bool allow_yaml_with_no_cpp{false};

  /** Allows Serializables to provide more key-value pairs than are present in
  the YAML data. In other words, the structs have default values that are left
  intact unless the YAML data provides a value. */
  bool allow_cpp_with_no_yaml{false};

  /** If set to true, when parsing a std::map the Archive will merge the YAML
  data into the destination, instead of replacing the std::map contents
  entirely. In other words, a visited std::map can have default values that are
  left intact unless the YAML data provides a value *for that specific key*. */
  bool retain_map_defaults{false};
};

}  // namespace yaml
}  // namespace drake

// TODO(jwnimmer-tri) Add a real formatter and deprecate the operator<<.
namespace fmt {
template <>
struct formatter<drake::yaml::LoadYamlOptions> : drake::ostream_formatter {};
}  // namespace fmt
