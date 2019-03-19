#pragma once

#include <vector>

#include "drake/automotive/maliput/api/rules/direction_usage_rule.h"
#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/rules/speed_limit_rule.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for querying "rules of the road".  This interface
/// provides access to static information about a road network (i.e.,
/// information determined prior to the beginning of a simulation).  Some
/// rule types may refer to additional dynamic information which will be
/// provided by other interfaces.  (For example, see RightOfWayRule.)
///
/// Concrete implementations of this interface shall be provided by
/// implementing the pure virtual methods declared in private scope.
class RoadRulebook {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadRulebook);

  virtual ~RoadRulebook() = default;

  /// Results of a FindRules() query.  Results are organized by type; an
  /// empty vector indicates no applicable rules of that type are known.
  struct QueryResults {
    std::vector<RightOfWayRule> right_of_way;
    std::vector<SpeedLimitRule> speed_limit;
    std::vector<rules::DirectionUsageRule> direction_usage;
  };

  /// Returns a QueryResults structure which contains any rules which are
  /// applicable to the provided `ranges`.
  ///
  /// `tolerance` is the acceptable linear-tolerance in longitudinal
  /// s-coordinate in each range and must be non-negative.  A non-zero
  /// `tolerance` makes the query more permissive.  However, a non-zero
  /// `tolerance` does not permit matching across BranchPoints (past the
  /// s-bounds of a Lane).
  ///
  /// @throws std::runtime_error if `tolerance` is negative.
  QueryResults FindRules(
      const std::vector<LaneSRange>& ranges, double tolerance) const {
    DRAKE_THROW_UNLESS(tolerance >= 0.);
    return DoFindRules(ranges, tolerance);
  }

  /// Returns the RightOfWayRule with the specified `id`.
  ///
  /// @throws std::out_of_range if `id` is unknown.
  RightOfWayRule GetRule(const RightOfWayRule::Id& id) const {
    return DoGetRule(id);
  }

  /// Returns the SpeedLimitRule with the specified `id`.
  ///
  /// @throws std::out_of_range if `id` is unknown.
  SpeedLimitRule GetRule(const SpeedLimitRule::Id& id) const {
    return DoGetRule(id);
  }

  /// Returns the DirectionUsageRule with the specified `id`.
  ///
  /// @throws std::out_of_range if `id` is unknown.
  DirectionUsageRule GetRule(const DirectionUsageRule::Id& id) const {
    return DoGetRule(id);
  }

 protected:
  RoadRulebook() = default;

 private:
  // @name NVI implementations of the public methods.
  // These must satisfy the constraints/invariants of the
  // corresponding public methods.
  //@{
  virtual QueryResults DoFindRules(
      const std::vector<LaneSRange>& ranges, double tolerance) const = 0;
  virtual RightOfWayRule DoGetRule(const RightOfWayRule::Id& id) const = 0;
  virtual SpeedLimitRule DoGetRule(const SpeedLimitRule::Id& id) const = 0;
  virtual DirectionUsageRule DoGetRule(
    const DirectionUsageRule::Id& id) const = 0;
  //@}
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
