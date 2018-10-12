#include "drake/automotive/maliput/api/rules/right_of_way_phase_ring.h"

#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

RightOfWayPhaseRing::RightOfWayPhaseRing(const Id& id,
    const std::vector<RightOfWayPhase>& phases) : id_(id) {
    DRAKE_THROW_UNLESS(phases.size() >= 1);
    for (const RightOfWayPhase& phase : phases) {
      // Construct index of phases by ID, ensuring uniqueness of ID's.
      auto result = phases_.emplace(phase.id(), phase);
      DRAKE_THROW_UNLESS(result.second);
    }
  }

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
