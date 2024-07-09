#pragma once

#include <map>
#include <set>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/common/sorted_pair.h"
#include "drake/multibody/parsing/detail_instanced_name.h"
#include "drake/multibody/tree/scoped_name.h"

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
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CollisionFilterGroupsImpl);

  CollisionFilterGroupsImpl();
  ~CollisionFilterGroupsImpl();

  bool operator==(const CollisionFilterGroupsImpl&) const;  // = default;

  void AddGroup(const T& name, const std::set<T>& members);

  void AddExclusionPair(const SortedPair<T>& pair);

  bool empty() const { return groups_.empty() && pairs_.empty(); }

  const std::map<T, std::set<T>>& groups() const { return groups_; }

  const std::set<SortedPair<T>>& exclusion_pairs() const { return pairs_; }

  std::string to_string() const;

  /* Merge two collision filter groups into one. This function is NOT part of
  the public-facing CollisionFilterGroups API; it's an internal detail.
  @param source a groups object to be merged into `this`.
  @pre The group names in `*this` and source must be disjoint sets. */
  void Merge(const CollisionFilterGroupsImpl& source);

  /* Copies this from type <T> to type <U>. This function is NOT part of the
  public-facing CollisionFilterGroups API; it's an internal detail. */
  template <typename U>
  auto Convert(const std::function<U(const T&)>& name_converter) const
      -> CollisionFilterGroupsImpl<U>;

 private:
  std::map<T, std::set<T>> groups_;
  std::set<SortedPair<T>> pairs_;
};

/* Copies `other` from type <InstancedName> to type <std::string>, using
the given plant to lookup model instance names. */
template <typename MultibodyPlantDeferred>
CollisionFilterGroupsImpl<std::string> ConvertInstancedNamesToStrings(
    const CollisionFilterGroupsImpl<InstancedName>& other,
    const MultibodyPlantDeferred& plant) {
  auto name_converter = [&plant](const InstancedName& instanced_name) {
    return instanced_name.index.has_value()
               ? std::string{ScopedName::Join(plant.GetModelInstanceName(
                                                  *instanced_name.index),
                                              instanced_name.name)
                                 .get_full()}
               : instanced_name.name;
  };
  return other.template Convert<std::string>(name_converter);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_FORMATTER_AS(typename T, drake::multibody::internal,
                   CollisionFilterGroupsImpl<T>, x, x.to_string())
