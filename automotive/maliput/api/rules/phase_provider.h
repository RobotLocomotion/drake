#pragma once

#include "drake/automotive/maliput/api/rules/phase.h"
#include "drake/automotive/maliput/api/rules/phase_ring.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

/// Abstract interface for providing the dynamic states (Phase::Id) of a
/// collection of PhaseRings.
class PhaseProvider {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PhaseProvider);

  virtual ~PhaseProvider() = default;

  /// Result returned by GetPhase().
  struct Result {
    /// Information about a subsequent phase.
    struct Next {
      /// ID of the next phase.
      Phase::Id id;
      /// If known, estimated time until the transition to the next phase.
      drake::optional<double> duration_until;
    };

    /// ID of the current phase.
    Phase::Id id;
    /// Information about the upcoming phase if a phase transition is
    /// anticipated.
    drake::optional<Next> next;
  };

  /// Gets the phase within a specified PhaseRing. Returns nullopt if
  /// @p id is unrecognized.
  optional<Result> GetPhase(const PhaseRing::Id& id) const {
    return DoGetPhase(id);
  }

 protected:
  PhaseProvider() = default;

 private:
  virtual optional<Result> DoGetPhase(const PhaseRing::Id& id)
      const = 0;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
