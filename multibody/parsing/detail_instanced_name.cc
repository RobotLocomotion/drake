#include "drake/multibody/parsing/detail_instanced_name.h"

#include <cstring>

namespace drake {
namespace multibody {
namespace internal {

std::string InstancedName::to_string() const {
  return fmt::format(
      "[{}, {}]", index.has_value() ? fmt::to_string(*index) : "nullopt", name);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
