#pragma once

#include <optional>
#include <string>

#include "drake/common/fmt_ostream.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {
namespace internal {

// Describes a name as found during parsing, together with index of the model
// where it was found. It purposely does not record the model by name, since
// model renaming via the plant could invalidate such names.
struct InstancedName {
  std::optional<ModelInstanceIndex> index;  // Empty means global name.
  std::string name;

  bool operator==(const InstancedName&) const = default;
  auto operator<=>(const InstancedName&) const = default;
};

template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const InstancedName& name) {
  os << fmt::format("[{}, {}]",
                    name.index.has_value()
                    ? fmt::to_string(*name.index)
                    : "nullopt",
                    name.name);
  return os;
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

// TODO(jwnimmer-tri) Add a real formatter and deprecate the operator<<.
namespace fmt {
template <>
struct formatter<drake::multibody::internal::InstancedName>
    : drake::ostream_formatter {};
}  // namespace fmt

