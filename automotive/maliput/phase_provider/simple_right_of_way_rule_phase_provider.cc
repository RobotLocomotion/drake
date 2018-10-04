#include "drake/automotive/maliput/phase_provider/simple_right_of_way_rule_phase_provider.h"

#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace phase_provider {

using api::rules::RightOfWayRulePhase;

void SimpleRightOfWayRulePhaseProvider::SetCurrentPhase(
    const RightOfWayRulePhase::Id& id) {
  DRAKE_THROW_UNLESS(!!GetPhase(id));
  current_phase_id_ = id;
}

optional<RightOfWayRulePhase::Id>
SimpleRightOfWayRulePhaseProvider::GetCurrentPhase() const {
  return current_phase_id_;
}

}  // namespace phase_provider
}  // namespace maliput
}  // namespace drake
