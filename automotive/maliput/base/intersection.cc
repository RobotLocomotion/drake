#include "drake/automotive/maliput/base/intersection.h"

namespace drake {
namespace maliput {

Intersection::Intersection(const Id& id,
                           const std::vector<api::rules::LaneSRange>& region,
                           const api::rules::RightOfWayPhaseRing* ring,
                           SimpleRightOfWayPhaseProvider* phase_provider)
    : api::Intersection(id, region, ring), phase_provider_(phase_provider) {
  DRAKE_THROW_UNLESS(phase_provider_ != nullptr);
}

const optional<api::rules::RightOfWayPhaseProvider::Result>
Intersection::Phase() const {
  return phase_provider_->GetPhase(ring()->id());
}

void Intersection::SetPhase(const api::rules::RightOfWayPhase::Id& phase_id) {
  phase_provider_->SetPhase(ring()->id(), phase_id);
}

}  // namespace maliput
}  // namespace drake
