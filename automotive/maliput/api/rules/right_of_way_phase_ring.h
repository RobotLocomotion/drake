#pragma once

#include <unordered_map>
#include <vector>

#include "drake/automotive/maliput/api/rules/right_of_way_phase.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

/// A set of mutually exclusive phases, e.g., that comprise the signalling
/// cycle for an intersection.
class RightOfWayPhaseRing final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(RightOfWayPhaseRing);

  /// Unique identifier for a RightOfWayPhaseRing.
  using Id = TypeSpecificIdentifier<class RightOfWayPhaseRing>;

  /// Constructs a RightOfWayPhaseRing.
  ///
  /// @param id the unique ID of this phase ring
  /// @param phases the phases within this ring.
  ///
  /// @throws std::exception if `phases` is empty, `phases` contains duplicate
  /// RightOfWayPhase::Id's, the phases define different sets of
  /// RightOfWayRule::Ids, or the phases define different sets of bulb states.
  RightOfWayPhaseRing(const Id& id, const std::vector<RightOfWayPhase>& phases);

  /// Returns the phase ring's identifier.
  const Id& id() const { return id_; }

  /// Returns the catalog of phases.
  const std::unordered_map<RightOfWayPhase::Id, RightOfWayPhase>& phases()
      const {
    return phases_;
  }

 private:
  Id id_;
  std::unordered_map<RightOfWayPhase::Id, RightOfWayPhase> phases_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
