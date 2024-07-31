#pragma once

#include <map>
#include <set>
#include <string>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/fmt.h"
#include "drake/common/sorted_pair.h"

namespace drake {
namespace multibody {

namespace internal {
template <typename T>
class CollisionFilterGroupsImpl;
}  // namespace internal

/** This is storage for parsed collision filter groups and group pairs. This
data may be useful to users needing to compose further collision filters in
code, without having to restate the data already captured in model files.

The contents of this object will be made up of names of collision filter groups
and bodies. By convention, the name strings are treated as scoped names.

Note that this object enforces few invariants on the data. In the expected
workflow, the parser will add groups and exclusion pairs found during
parsing. The only condition checked here is that a group with a given name is
only added once.
*/
class CollisionFilterGroups {
 public:
  DRAKE_DECLARE_COPY_AND_MOVE_AND_ASSIGN(CollisionFilterGroups);

  CollisionFilterGroups();
  ~CollisionFilterGroups();

  bool operator==(const CollisionFilterGroups&) const;

  /** Adds a new collision filter group.
  @param name the fully-qualified scoped name of the group being defined.
  @param members the fully-qualified scoped names of the member bodies.
  @pre name is not already a defined group in this object.
  */
  void AddGroup(const std::string& name, const std::set<std::string>& members);

  /** Adds an exclusion pair between two collision filter groups.
  @param pair a pair of fully-qualified scoped names of groups.
  @note A pair can consist of the same name twice, which means the pair defines
  a rule where all members of the group exclude each other. Adding an already
  defined pair does nothing.
  */
  void AddExclusionPair(const SortedPair<std::string>& pair);

  /** @returns true iff both groups() and exclusion_pairs() are empty. */
  bool empty() const;

  /** @returns the groups stored by prior calls to AddGroup(). */
  const std::map<std::string, std::set<std::string>>& groups() const;

  /** @returns the pairs stored by prior calls to AddExclusionPair(). */
  const std::set<SortedPair<std::string>>& exclusion_pairs() const;

  /** @returns a multi-line human-readable representation of this' contents. */
  std::string to_string() const;

#ifndef DRAKE_DOXYGEN_CXX
  /* (Internal use only) Internal move-constructor for efficiency. */
  explicit CollisionFilterGroups(
      internal::CollisionFilterGroupsImpl<std::string>&&);
#endif

 private:
  // To reduce implementation complexity, the impl_ pointer must never be null.
  // Otherwise, every method would need to check for null and implement
  // alternative behavior.
  copyable_unique_ptr<internal::CollisionFilterGroupsImpl<std::string>> impl_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::multibody, CollisionFilterGroups, x, x.to_string())
