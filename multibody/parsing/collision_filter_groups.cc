#include "drake/multibody/parsing/collision_filter_groups.h"

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
  groups_[name_str] = members;
}
void CollisionFilterGroups::AddExclusionPair(
    const SortedPair<std::string>& pair) {
  pairs_.insert(pair);
}

void CollisionFilterGroups::Clear() {
  groups_.clear();
  pairs_.clear();
}

bool CollisionFilterGroups::empty() const {
  return groups_.empty() && pairs_.empty();
}

const std::map<std::string, std::set<std::string>>&
CollisionFilterGroups::groups() const { return groups_; }

const std::set<SortedPair<std::string>>&
CollisionFilterGroups::exclusion_pairs() const { return pairs_; }

std::ostream& operator<<(std::ostream& os, const CollisionFilterGroups& g) {
  os << "\nCollision filter groups:\n";
  for (const auto& item : g.groups()) {
    os << fmt::format("    {}\n", item.first);
    for (const auto& value : item.second) {
      os << fmt::format("        {}\n", value);
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
