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

  /// Result returned by GetCurrentPhase().
  struct Result {
    /// Information about a subsequent phase.
    struct Next {
      /// ID of the next phase.
      RightOfWayRulePhase::Id id;
      /// If known, estimated time until the transition to the next phase.
      drake::optional<double> duration_until;
    };

    /// ID of the current phase.
    RightOfWayRulePhase::Id id;
    /// Information about the upcoming phase if a phase transition is
    /// anticipated.
    drake::optional<Next> next;
  };

  /// Adds a phase to this provider. The phase ID must not exist within this
  /// provider.
  void AddPhase(const RightOfWayRulePhase& phase);

  /// Gets a phase based on its ID. This enables users to obtain any phase
  /// regardless of whether it's the current phase.
  const optional<const RightOfWayRulePhase&> GetPhase(
      const RightOfWayRulePhase::Id& id) const;

  /// Gets the ID of the current phase. Returns nullopt if no current phase is
  /// specified.
  virtual optional<Result> GetCurrentPhase() const = 0;

 protected:
  RightOfWayRulePhaseProvider() = default;

 private:
  std::map<RightOfWayRulePhase::Id, RightOfWayRulePhase> phases_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
