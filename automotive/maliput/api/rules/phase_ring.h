#pragma once

#include <unordered_map>
#include <vector>

#include "drake/automotive/maliput/api/rules/phase.h"
#include "drake/automotive/maliput/api/type_specific_identifier.h"
#include "drake/common/drake_copyable.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

/// A set of mutually exclusive phases, e.g., that comprise the signaling
/// cycle for an intersection.
class PhaseRing final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PhaseRing);

  /// Unique identifier for a PhaseRing.
  using Id = TypeSpecificIdentifier<class PhaseRing>;

  /// Constructs a PhaseRing.
  ///
  /// @param id the unique ID of this phase ring
  /// @param phases the phases within this ring.
  ///
  /// @throws std::exception if `phases` is empty, `phases` contains duplicate
  /// Phase::Id's, the phases define different sets of
  /// RightOfWayRule::Ids, or the phases define different sets of bulb states.
  PhaseRing(const Id& id, const std::vector<Phase>& phases);

  /// Returns the phase ring's identifier.
  const Id& id() const { return id_; }

  /// Returns the catalog of phases.
  const std::unordered_map<Phase::Id, Phase>& phases() const {
    return phases_;
  }

 private:
  Id id_;
  std::unordered_map<Phase::Id, Phase> phases_;
};

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
