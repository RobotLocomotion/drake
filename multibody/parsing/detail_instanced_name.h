#pragma once

#include <optional>
#include <string>

#include "drake/common/fmt.h"
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

  std::string to_string() const {
    return fmt::format("[{}, {}]",
                       index.has_value() ? fmt::to_string(*index) : "nullopt",
                       name);
  }
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::multibody::internal, InstancedName, x,
                   x.to_string())
