#pragma once

#include <unordered_map>

#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

using RuleStates =
    std::unordered_map<RightOfWayRule::Id, RightOfWayRule::State::Id>;

/// A group of RightOfWayRule instances and their states. It models coupling
/// between these rules due to, for example, spatial co-location at
/// intersections.
class RightOfWayPhase final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RightOfWayPhase);

  /// Unique identifier for a RightOfWayPhase.
  using Id = TypeSpecificIdentifier<RightOfWayPhase>;

  /// Constructs a RightOfWayPhase.
  ///
  /// @param id the unique ID of the phase (in the RightOfWayRulePhaseRing)
  /// @param rule_states the RightOfWayRules and their states when the phase is
  /// applied, e.g., to an intersection.
  RightOfWayPhase(const Id& id, const RuleStates& rule_states);

  /// Returns the phase's identifier.
  const Id& id() const { return id_; }

  /// Returns the phase's RightOfWayRule instances and their states.
  const RuleStates& rule_states() const { return rule_states_; }

 private:
  Id id_;
  RuleStates rule_states_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
