#pragma once

#include <bitset>
#include <string>
#include <type_traits>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/fmt.h"
#include "drake/planning/sampling_based/dev/path_planning_error.h"

namespace drake {
namespace planning {

/** Like a std::set<PathPlanningError> but much more compact. */
class PathPlanningErrors {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PathPlanningErrors);

  /** Constructs an empty set (i.e., represents success). */
  PathPlanningErrors() = default;

  /** Constructs a set with `error` as the sole element. */
  explicit PathPlanningErrors(PathPlanningError error) { insert(error); }

  /** Adds the given `error` to this set. If `error` was already a member of
  this set, does nothing. */
  void insert(PathPlanningError error) { bits_.set(cast(error)); }

  /** Returns true if the given `error` is a member of this set. */
  bool contains(PathPlanningError error) const {
    return bits_.test(cast(error));
  }

  /** Returns true if this set is empty (i.e., represents success). */
  bool empty() const { return bits_.none(); }

  /** Converts this set to a vector (typically used for iteration). */
  std::vector<PathPlanningError> to_vector() const;

 private:
  using ErrorAsInt = std::underlying_type_t<PathPlanningError>;
  static constexpr ErrorAsInt cast(PathPlanningError error) {
    return static_cast<ErrorAsInt>(error);
  }
  static constexpr size_t N =
      1 + static_cast<ErrorAsInt>(internal::kMaxPathPlanningError);
  std::bitset<N> bits_;
};

/** Returns a string representation of `errors` for debugging output. */
std::string to_string(const PathPlanningErrors& errors);

}  // namespace planning
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::planning, PathPlanningErrors, x,
                   drake::planning::to_string(x))
