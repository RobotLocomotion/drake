#include "drake/multibody/parsing/collision_filter_groups.h"

#include <utility>

#include <fmt/format.h>

namespace drake {
namespace multibody {

bool CollisionFilterGroups::operator==(const CollisionFilterGroups& rhs) const {
  return std::tie(groups_, pairs_) == std::tie(rhs.groups_, rhs.pairs_);
}

void CollisionFilterGroups::AddGroup(std::string_view name,
                                     const std::set<std::string> members) {
  std::string name_str(name);
  DRAKE_DEMAND(!groups_.contains(name_str));
  groups_.insert({name_str, std::move(members)});
}

void CollisionFilterGroups::AddExclusionPair(
    const SortedPair<std::string> pair) {
  pairs_.insert(std::move(pair));
}

bool CollisionFilterGroups::empty() const {
  return groups_.empty() && pairs_.empty();
}

std::ostream& operator<<(std::ostream& os, const CollisionFilterGroups& g) {
  os << "\nCollision filter groups:\n";
  for (const auto&  [name, members] : g.groups()) {
    os << fmt::format("    {}\n", name);
    for (const auto& member : members) {
      os << fmt::format("        {}\n", member);
    }
  }
  os << "Collision filter exclusion pairs:\n";
  for (const auto& pair : g.exclusion_pairs()) {
    os << fmt::format("    {}, {}\n", pair.first(), pair.second());
  }
  return os;
}

}  // namespace multibody
}  // namespace drake
