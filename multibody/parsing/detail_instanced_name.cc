#include "drake/multibody/parsing/detail_instanced_name.h"

#include <cstring>

namespace drake {
namespace multibody {
namespace internal {

// Apple Xcode fails to correctly implement <=> for either
// std::optional (prior to 15.3) or std::string (prior to 15.2
// maybe?), so the default implementation of <=> here gets
// "implicitly deleted", leading to a train of compile errors. So,
// until Ventura support ends, write an explicit implementation
// instead.
//
// TODO(rpoyner-tri): replace explicit implementation with compiler
// default once macOS Ventura support ends.
std::strong_ordering InstancedName::operator<=>(
    const InstancedName& that) const {
  if (index.has_value() && that.index.has_value()) {
    if (auto cmp = (*index <=> *that.index); cmp != 0) {
      return cmp;
    }
  }
  if (auto cmp = (index.has_value() <=> that.index.has_value()); cmp != 0) {
    return cmp;
  }
  return std::strcmp(name.c_str(), that.name.c_str()) <=> 0;
}

std::string InstancedName::to_string() const {
  return fmt::format(
      "[{}, {}]", index.has_value() ? fmt::to_string(*index) : "nullopt", name);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
