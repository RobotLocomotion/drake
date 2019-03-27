#pragma once

#include <unordered_map>
#include <vector>

#include "drake/automotive/maliput/api/rules/right_of_way_phase.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

/// A set of mutually exclusive phases, e.g., that comprise the signaling
/// cycle for an intersection.
class PhaseRing final {
 public:
  /// Holds a "next phase" specification. This is one of the phases that could
  /// be next after the current phase ends.
  struct NextPhase {
    /// The ID of the next phase.
    RightOfWayPhase::Id id;

    /// The default time before transitioning to the next phase. This is
    /// relative to when the current phase began. It is just a recommendation,
    /// the actual duration is determined by the RightOfWayPhaseProvider and
    /// may depend on events like a vehicle arriving at a left-turn lane or a
    /// pedestrian hitting a crosswalk button.
    optional<double> duration_until;
  };

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PhaseRing);

  /// Unique identifier for a PhaseRing.
  using Id = TypeSpecificIdentifier<class PhaseRing>;

  /// Constructs a PhaseRing.
  ///
  /// @param id the unique ID of this phase ring
  /// @param phases the phases within this ring.
  /// @param next_phases specifies, for each phase, possible next phases. This
  /// can be nullopt, in which case, no next phases will be specified.
  ///
  /// @throws std::exception if `phases` is empty, `phases` contains duplicate
  /// RightOfWayPhase::Id's, the phases define different sets of
  /// RightOfWayRule::Ids, or the phases define different sets of bulb states.
  /// Also if `next_phases` is not nullopt but does not define the possible next
  /// phases of every phase in `phases` or defines the next phases of a phase
  /// that is not in `phases`.
  PhaseRing(const Id& id, const std::vector<RightOfWayPhase>& phases,
            optional<const std::unordered_map<RightOfWayPhase::Id,
                                              std::vector<NextPhase>>>
                    next_phases = nullopt);

  /// Returns the phase ring's identifier.
  const Id& id() const { return id_; }

  /// Returns the catalog of phases.
  const std::unordered_map<RightOfWayPhase::Id, RightOfWayPhase>& phases()
      const {
    return phases_;
  }

  const std::unordered_map<RightOfWayPhase::Id, std::vector<NextPhase>>&
  next_phases() const {
    return next_phases_;
  }

 private:
  Id id_;
  std::unordered_map<RightOfWayPhase::Id, RightOfWayPhase> phases_;
  std::unordered_map<RightOfWayPhase::Id, std::vector<NextPhase>> next_phases_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
