#include "drake/multibody/parsing/detail_collision_filter_groups_impl.h"

#include <sstream>

#include "drake/common/fmt.h"

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
  DRAKE_THROW_UNLESS(!groups_.contains(name));
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

template <typename T>
void CollisionFilterGroupsImpl<T>::Merge(
    const CollisionFilterGroupsImpl& source) {
  for (const auto& [group_name, members] : source.groups()) {
    this->AddGroup(group_name, members);
  }
  for (const auto& exclusion_pair : source.exclusion_pairs()) {
    this->AddExclusionPair(exclusion_pair);
  }
}

template <typename T>
template <typename U>
auto CollisionFilterGroupsImpl<T>::Convert(
    const std::function<U(const T&)>& name_converter) const
    -> CollisionFilterGroupsImpl<U> {
  CollisionFilterGroupsImpl<U> result;
  for (const auto& [group_name, members] : groups()) {
    std::set<U> result_members;
    for (const auto& member : members) {
      result_members.insert(name_converter(member));
    }
    result.AddGroup(name_converter(group_name), result_members);
  }
  for (const auto& [a, b] : exclusion_pairs()) {
    result.AddExclusionPair({name_converter(a), name_converter(b)});
  }
  return result;
}

// Instantiations used by the parser.
template class CollisionFilterGroupsImpl<std::string>;
template class CollisionFilterGroupsImpl<InstancedName>;
template auto CollisionFilterGroupsImpl<std::string>::Convert(
    const std::function<InstancedName(const std::string&)>& name_converter)
    const -> CollisionFilterGroupsImpl<InstancedName>;
template auto CollisionFilterGroupsImpl<InstancedName>::Convert(
    const std::function<std::string(const InstancedName&)>& name_converter)
    const -> CollisionFilterGroupsImpl<std::string>;

// Instantiations used by our unit test.
template class CollisionFilterGroupsImpl<double>;
template class CollisionFilterGroupsImpl<int>;
template auto CollisionFilterGroupsImpl<int>::Convert(
    const std::function<double(const int&)>& name_converter) const
    -> CollisionFilterGroupsImpl<double>;
template auto CollisionFilterGroupsImpl<double>::Convert(
    const std::function<int(const double&)>& name_converter) const
    -> CollisionFilterGroupsImpl<int>;

}  // namespace internal
}  // namespace multibody
}  // namespace drake
