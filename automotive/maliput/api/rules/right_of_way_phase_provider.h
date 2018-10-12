#pragma once

#include <map>

#include "drake/automotive/maliput/api/rules/right_of_way_phase.h"
#include "drake/automotive/maliput/api/rules/right_of_way_phase_ring.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

/// Provides the "current" RightOfWayPhase within a particular
/// RightOfWayPhaseRing. The "current" phase is the one whose RuleStates are
/// in effect. Child classes can implement custom policies for transitioning
/// between different RightOfWayPhase instances.
class RightOfWayPhaseProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RightOfWayPhaseProvider);

  virtual ~RightOfWayPhaseProvider() = default;

  /// Result returned by GetPhase().
  struct Result {
    /// Information about a subsequent phase.
    struct Next {
      /// ID of the next phase.
      RightOfWayPhase::Id id;
      /// If known, estimated time until the transition to the next phase.
      drake::optional<double> duration_until;
    };

    /// ID of the current phase.
    RightOfWayPhase::Id id;
    /// Information about the upcoming phase if a phase transition is
    /// anticipated.
    drake::optional<Next> next;
  };

  /// Gets the "current" phase within a specified RightOfWayPhaseRing.
  const optional<Result> GetPhase(const RightOfWayPhaseRing::Id& id) const {
    return DoGetPhase(id);
  }

 protected:
  RightOfWayPhaseProvider() = default;

 private:
  virtual optional<Result> DoGetPhase(const RightOfWayPhaseRing::Id& id)
      const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
