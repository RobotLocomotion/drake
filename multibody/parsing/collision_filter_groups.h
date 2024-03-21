#pragma once

#include <map>
#include <set>
#include <string>
#include <string_view>

#include "drake/common/drake_copyable.h"
#include "drake/common/fmt_ostream.h"
#include "drake/common/sorted_pair.h"

namespace drake {
namespace multibody {

/** This is storage for parsed collision filter groups and group pairs. This
data may be useful to users needing to compose further collision filters in
code, without having to restate the data already captured in model files.

The contents of this object will be made up of fully-qualified scoped names of
collision filter groups and bodies.

Note that this object enforces few invariants on the data. In the expected
workflow, the parser will add groups and exclusion pairs found during
parsing. The only condition checked here is that a group with a given
fully-qualified name is only added once.
*/
class CollisionFilterGroups {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CollisionFilterGroups)

  CollisionFilterGroups() = default;

  bool operator==(const CollisionFilterGroups&) const;

  /** Adds a new collision filter group.
  @param name the fully-qualified scoped name of the group being defined.
  @param members the fully-qualified scoped names of the member bodies.
  @pre name is not already a defined group in this object.
  */
  void AddGroup(std::string_view name,
                const std::set<std::string> members);

  /** Adds an exclusion pair between two collision filter groups.
  @param pair a pair of fully-qualified scoped names of groups.
  @note A pair can consist of the same name twice, which means the pair defines
  a rule where all members of the group exclude each other. Adding an already
  defined pair does nothing.
  */
  void AddExclusionPair(const SortedPair<std::string> pair);

  /** @returns true iff both groups() and exclusion_pairs() are empty. */
  bool empty() const;

  /** @returns the groups stored by prior calls to AddGroup(). */
  const std::map<std::string, std::set<std::string>>& groups() const {
    return groups_;
  }

  /** @returns the pairs stored by prior calls to AddExclusionPair(). */
  const std::set<SortedPair<std::string>>& exclusion_pairs() const {
    return pairs_;
  }

 private:
  std::map<std::string, std::set<std::string>> groups_;
  std::set<SortedPair<std::string>> pairs_;
};

/** Emits a multi-line, human-readable report of CollisionFilterGroups
content. */
std::ostream& operator<<(std::ostream& os, const CollisionFilterGroups& g);

}  // namespace multibody
}  // namespace drake

// TODO(jwnimmer-tri) Add a real formatter and deprecate the operator<<.
namespace fmt {
template <>
struct formatter<drake::multibody::CollisionFilterGroups>
    : drake::ostream_formatter {};
}  // namespace fmt
