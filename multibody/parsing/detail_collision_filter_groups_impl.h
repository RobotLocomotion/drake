#pragma once

#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/fmt.h"
#include "drake/common/sorted_pair.h"
#include "drake/multibody/parsing/detail_instanced_name.h"

namespace drake {
namespace multibody {
namespace internal {

/* Implementation of multibody::CollisionFilterGroups. See that class for full
documentation.

@tparam T the type used for naming groups and members, which must be either
`std::string` or `InstancedName`. (For the purposes of our own unit test, we
also allow `int` and `double` but those are not actually used by the Parser.)

Internal to the parser, we use CollisionFilterGroupsImpl<InstanceName> so that
we can defend against model instance renames. When the user asks for the groups,
we copy it to a CollisionFilterGroupsImpl<std::string> for their convenience. */
template <typename T>
class CollisionFilterGroupsImpl {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CollisionFilterGroupsImpl)

  CollisionFilterGroupsImpl();
  ~CollisionFilterGroupsImpl();

  bool operator==(const CollisionFilterGroupsImpl&) const;  // = default;

  void AddGroup(const T& name, const std::set<T>& members);

  void AddExclusionPair(const SortedPair<T>& pair);

  bool empty() const { return groups_.empty() && pairs_.empty(); }

  const std::map<T, std::set<T>>& groups() const { return groups_; }

  const std::set<SortedPair<T>>& exclusion_pairs() const { return pairs_; }

  std::string to_string() const;

 private:
  std::map<T, std::set<T>> groups_;
  std::set<SortedPair<T>> pairs_;
};

/* Merge two collision filter groups into one. The groups may use different
types for names; the caller must also provide a conversion function from
names in the source to names in the destination.
@param destination the groups object to receive all the data.
@param source a groups object to be merged into the destination.
@param convert a function to convert name objects used by the source to name
objects used by the destination.
@tparam T the type for name objects used in the destination.
@tparam U the type for name objects used in the source.
@pre The group names in destination and source must be disjoint sets.
*/
template <typename T, typename U>
void MergeCollisionFilterGroups(CollisionFilterGroupsImpl<T>* destination,
                                const CollisionFilterGroupsImpl<U>& source,
                                std::function<T(const U&)> convert) {
  for (const auto& [group_name, members] : source.groups()) {
    std::set<T> destination_members;
    for (const auto& member : members) {
      destination_members.insert(convert(member));
    }
    destination->AddGroup(convert(group_name), destination_members);
  }

  for (const auto& pair : source.exclusion_pairs()) {
    destination->AddExclusionPair(
        {convert(pair.first()), convert(pair.second())});
  }
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_FORMATTER_AS(typename T, drake::multibody::internal,
                   CollisionFilterGroupsImpl<T>, x, x.to_string())
