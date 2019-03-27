#pragma once

#include <unordered_map>

#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/rules/traffic_lights.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

/// A mapping from a RightOfWayRule::Id to a RightOfWayRule::State::Id. Just an
/// alias for user convenience.
using RuleStates =
    std::unordered_map<RightOfWayRule::Id, RightOfWayRule::State::Id>;

/// A mapping from a Bulb::Id to a BulbState. Just an alias for user
/// convenience.
using BulbStates = std::unordered_map<Bulb::Id, BulbState>;

/// A group of RightOfWayRule instances and their states. It models coupling
/// between these rules due to, for example, spatial co-location at
/// intersections.
class Phase final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Phase);

  /// Unique identifier for a Phase.
  using Id = TypeSpecificIdentifier<Phase>;

  /// Constructs a Phase.
  ///
  /// @param id the unique ID of the phase (in the RightOfWayRulePhaseRing)
  ///
  /// @param rule_states the RightOfWayRules and their states when the phase is
  /// applied, e.g., to an intersection.
  ///
  /// @param bulb_states the states of the bulbs when this phase is applied,
  /// e.g., to an intersection.
  Phase(const Id& id, const RuleStates& rule_states,
        optional<BulbStates> bulb_states = nullopt);

  /// Returns the phase's identifier.
  const Id& id() const { return id_; }

  /// Returns the phase's RightOfWayRule instances and their states.
  const RuleStates& rule_states() const { return rule_states_; }

  /// Returns the phase's bulb states.
  const optional<BulbStates>& bulb_states() const { return bulb_states_; }

 private:
  Id id_;
  RuleStates rule_states_;
  optional<BulbStates> bulb_states_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
