#include "drake/automotive/maliput/phase_provider/simple_right_of_way_rule_phase_provider.h"

#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace phase_provider {

using api::rules::RightOfWayRulePhase;
using api::rules::RightOfWayRulePhaseProvider;

void SimpleRightOfWayRulePhaseProvider::SetCurrentPhase(
    const RightOfWayRulePhase::Id& id) {
  DRAKE_THROW_UNLESS(!!GetPhase(id));
  current_phase_id_ = id;
}

void SimpleRightOfWayRulePhaseProvider::SetNextPhase(
    const RightOfWayRulePhase::Id& id, optional<double> duration_until) {
  DRAKE_THROW_UNLESS(!!GetPhase(id));
  next_phase_id_ = id;
  duration_until_ = duration_until;
}

void SimpleRightOfWayRulePhaseProvider::ResetNextPhase() {
  next_phase_id_ = nullopt;
  duration_until_ = nullopt;
}

optional<RightOfWayRulePhaseProvider::Result>
SimpleRightOfWayRulePhaseProvider::GetCurrentPhase() const {
  if (current_phase_id_ == nullopt) {
    return nullopt;
  } else if (next_phase_id_ == nullopt) {
    return RightOfWayRulePhaseProvider::Result{current_phase_id_.value(),
                                               nullopt};
  } else {
    return RightOfWayRulePhaseProvider::Result{current_phase_id_.value(),
        RightOfWayRulePhaseProvider::Result::Next{next_phase_id_.value(),
                                                  duration_until_}};
  }
}

}  // namespace phase_provider
}  // namespace maliput
}  // namespace drake
