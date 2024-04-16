#pragma once

#include <map>
#include <set>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/common/fmt_ostream.h"
#include "drake/common/sorted_pair.h"

namespace drake {
namespace multibody {

/** This is storage for parsed collision filter groups and group pairs. This
data may be useful to users needing to compose further collision filters in
code, without having to restate the data already captured in model files.

The contents of this object will be made up of names of collision filter groups
and bodies. The type of the names is provided by a template parameter.

Note that this object enforces few invariants on the data. In the expected
workflow, the parser will add groups and exclusion pairs found during
parsing. The only condition checked here is that a group with a given name is
only added once.

@tparam T the type used for naming groups and members.
*/
template <typename T>
class CollisionFilterGroupsBase {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CollisionFilterGroupsBase)

  CollisionFilterGroupsBase
  () = default;

  bool operator==(const CollisionFilterGroupsBase&) const = default;
  auto operator<=>(const CollisionFilterGroupsBase&) const = default;

  /** Adds a new collision filter group.
  @param name the fully-qualified scoped name of the group being defined.
  @param members the fully-qualified scoped names of the member bodies.
  @pre name is not already a defined group in this object.
  */
  void AddGroup(const T& name, const std::set<T> members) {
    DRAKE_DEMAND(!groups_.contains(name));
    groups_.insert({name, std::move(members)});
  }


  /** Adds an exclusion pair between two collision filter groups.
  @param pair a pair of fully-qualified scoped names of groups.
  @note A pair can consist of the same name twice, which means the pair defines
  a rule where all members of the group exclude each other. Adding an already
  defined pair does nothing.
  */
  void AddExclusionPair(const SortedPair<T>& pair) {
    pairs_.insert(pair);
  }


  /** @returns true iff both groups() and exclusion_pairs() are empty. */
  bool empty() const {
    return groups_.empty() && pairs_.empty();
  }


  /** @returns the groups stored by prior calls to AddGroup(). */
  const std::map<T, std::set<T>>& groups() const {
    return groups_;
  }

  /** @returns the pairs stored by prior calls to AddExclusionPair(). */
  const std::set<SortedPair<T>>& exclusion_pairs() const {
    return pairs_;
  }

 private:
  std::map<T, std::set<T>> groups_;
  std::set<SortedPair<T>> pairs_;
};

namespace internal {

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
void MergeCollisionFilterGroups(
    CollisionFilterGroupsBase<T>* destination,
    const CollisionFilterGroupsBase<U>& source,
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

/** Emits a multi-line, human-readable report of CollisionFilterGroups
content. */
template <typename T>
std::ostream& operator<<(std::ostream& os,
                         const CollisionFilterGroupsBase<T>& g) {
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

/** Collision filter groups that use string names. By convention, the strings
 * are treated as scoped names. */
using CollisionFilterGroups = CollisionFilterGroupsBase<std::string>;

}  // namespace multibody
}  // namespace drake

// TODO(jwnimmer-tri) Add a real formatter and deprecate the operator<<.
namespace fmt {
template <>
struct formatter<drake::multibody::CollisionFilterGroups>
    : drake::ostream_formatter {};
}  // namespace fmt
