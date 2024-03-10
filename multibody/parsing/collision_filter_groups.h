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

/** This is storage for computed collision filter group results from parsing.

The contents of this object will be made up of full-qualified scoped names of
collision filter groups and bodies.

Note that this object does not enforce any invariants on any of the data. The
consistency of the data is provided by parser's use of this object.

For objects of this class owned by the parser, it is expected that only the
parser will provide new data, via AddGroup() and AddExclusionPair(). Users of
the parser would typically copy the data for their own use, and optionally
Clear() the parser's object.
*/
class CollisionFilterGroups {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CollisionFilterGroups)

  CollisionFilterGroups() = default;

  bool operator==(const CollisionFilterGroups&) const;

  /** Adds a new collison filter group.
  @param name the fully qualified scoped name of the group being defined.
  @param members the fully qualified scoped names of the member bodies.
  @pre name is not already a defined group in this object.
  */
  void AddGroup(std::string_view name,
                const std::set<std::string> members);

  /** Adds an exclusion pair between two collision filter groups.
  @param pair a pair of fully qualified scope names of groups.
  @note A pair can consist of the same name twice, which means the pair defines
  a rule where all members of the group exclude each other. Adding an already
  defined pair does nothing.
  */
  void AddExclusionPair(const SortedPair<std::string>& pair);

  /** Clears the contents of this object. */
  void Clear();

  /** @returns true iff both groups() and exclusion_pairs() are empty. */
  bool empty() const;

  /** @returns the groups stored by prior calls to AddGroup(). */
  const std::map<std::string, std::set<std::string>>& groups() const;

  /** @returns the pairs stored by prior calls to AddExclusionPair(). */
  const std::set<SortedPair<std::string>>& exclusion_pairs() const;

 private:
  std::map<std::string, std::set<std::string>> groups_;
  std::set<SortedPair<std::string>> pairs_;
};

/** Emits a multi-line, human-readable report of groups content. */
std::ostream& operator<<(std::ostream& os, const CollisionFilterGroups& g);

}  // namespace multibody
}  // namespace drake

// TODO(jwnimmer-tri) Add a real formatter and deprecate the operator<<.
namespace fmt {
template <>
struct formatter<drake::multibody::CollisionFilterGroups>
    : drake::ostream_formatter {};
}  // namespace fmt
