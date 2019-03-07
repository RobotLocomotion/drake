#pragma once

#include "drake/automotive/maliput/api/rules/right_of_way_phase.h"
#include "drake/automotive/maliput/api/rules/right_of_way_phase_ring.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for providing the dynamic states (RightOfWayPhase::Id) of
/// a collection of RightOfWayPhaseRings.
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

  /// Gets the phase within a specified RightOfWayPhaseRing. Returns nullopt if
  /// @p id is unrecognized.
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
