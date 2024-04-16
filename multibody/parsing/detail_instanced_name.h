#pragma once

#include <compare>
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

#if defined(__apple_build_version__) && (__apple_build_version__ < 15000000)
  // Apple clang 14 fails to correctly implement <=> for either std::optional
  // or std::string, so the default implementation of <=> here gets "implicitly
  // deleted", leading to a train of compile errors. In that case, write an
  // explicit implementation instead.
  auto operator<=>(const InstancedName& that) const {
    if (index.has_value() && that.index.has_value()) {
      if (auto cmp = (*index <=> *that.index); cmp != 0) {
        return cmp;
      }
    }
    if (auto cmp = (index.has_value() <=> that.index.has_value()); cmp != 0) {
      return cmp;
    }
    return name <=> that.name;
  }
#else
  auto operator<=>(const InstancedName&) const = default;
#endif

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
