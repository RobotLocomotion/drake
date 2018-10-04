#pragma once

#include <map>

#include "drake/automotive/maliput/api/rules/right_of_way_rule.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

using RuleStates = std::map<RightOfWayRule::Id, RightOfWayRule::State::Id>;

/// A group of RightOfWayRule instances and their states. It models coupling
/// between these rules due to, for example, spatial co-location at
/// intersections.
struct RightOfWayRulePhase {
  using Id = TypeSpecificIdentifier<RightOfWayRulePhase>;
  explicit RightOfWayRulePhase(const Id& id_in) : id(id_in) {}
  Id id;
  RuleStates rule_states;
};

/// A provider of RightOfWayRulePhase instances. This acts as both a container
/// for holding these phases and a tracker of which phase is "current", i.e.,
/// the phase whose RuleStates are currently in effect. Child classes can
/// implement custom policies for transitioning between different
/// RightOfWayRulePhase instances.
class RightOfWayRulePhaseProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RightOfWayRulePhaseProvider);

  virtual ~RightOfWayRulePhaseProvider() = default;

  /// Adds a phase to this provider. The phase ID must not exist within this
  /// provider.
  void AddPhase(const RightOfWayRulePhase& phase);

  /// Gets a phase based on its ID.
  const optional<const RightOfWayRulePhase&> GetPhase(
      const RightOfWayRulePhase::Id& id) const;

  /// Gets the ID of the current phase.
  virtual optional<RightOfWayRulePhase::Id> GetCurrentPhase() const = 0;

 protected:
  RightOfWayRulePhaseProvider() = default;

 private:
  std::map<RightOfWayRulePhase::Id, RightOfWayRulePhase> phases_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
