#include "drake/multibody/parsing/detail_collision_filter_groups_impl.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
CollisionFilterGroupsImpl<T>::CollisionFilterGroupsImpl() = default;

template <typename T>
CollisionFilterGroupsImpl<T>::~CollisionFilterGroupsImpl() = default;

template <typename T>
bool CollisionFilterGroupsImpl<T>::operator==(
    const CollisionFilterGroupsImpl&) const = default;

template <typename T>
void CollisionFilterGroupsImpl<T>::AddGroup(const T& name,
                                            const std::set<T>& members) {
  DRAKE_DEMAND(!groups_.contains(name));
  groups_.insert({name, members});
}

template <typename T>
void CollisionFilterGroupsImpl<T>::AddExclusionPair(const SortedPair<T>& pair) {
  pairs_.insert(pair);
}

template <typename T>
std::string CollisionFilterGroupsImpl<T>::to_string() const {
  std::stringstream ss;
  ss << "\nCollision filter groups:\n";
  for (const auto& [name, members] : groups()) {
    ss << fmt::format("    {}\n", name);
    for (const auto& member : members) {
      ss << fmt::format("        {}\n", member);
    }
  }
  ss << "Collision filter exclusion pairs:\n";
  for (const auto& pair : exclusion_pairs()) {
    ss << fmt::format("    {}, {}\n", pair.first(), pair.second());
  }
  return ss.str();
}

// Instantiations used by the parser.
template class CollisionFilterGroupsImpl<std::string>;
template class CollisionFilterGroupsImpl<InstancedName>;

// Instantiations used by our unit test.
template class CollisionFilterGroupsImpl<double>;
template class CollisionFilterGroupsImpl<int>;

}  // namespace internal
}  // namespace multibody
}  // namespace drake
