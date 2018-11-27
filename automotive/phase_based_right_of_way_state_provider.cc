#include "drake/automotive/phase_based_right_of_way_state_provider.h"

#include "drake/automotive/maliput/api/rules/right_of_way_phase.h"
#include "drake/automotive/maliput/api/rules/right_of_way_phase_ring.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

using maliput::api::rules::RightOfWayPhase;
using maliput::api::rules::RightOfWayPhaseBook;
using maliput::api::rules::RightOfWayPhaseProvider;
using maliput::api::rules::RightOfWayPhaseRing;
using maliput::api::rules::RightOfWayRule;
using maliput::api::rules::RightOfWayStateProvider;

PhaseBasedRightOfWayStateProvider::PhaseBasedRightOfWayStateProvider(
    const RightOfWayPhaseBook* phase_book,
    const RightOfWayPhaseProvider* phase_provider)
    : phase_book_(phase_book),
      phase_provider_(phase_provider) {
  DRAKE_DEMAND(phase_book_ != nullptr && phase_provider != nullptr);
}

optional<RightOfWayStateProvider::Result>
PhaseBasedRightOfWayStateProvider::DoGetState(const RightOfWayRule::Id& rule_id)
    const {
  optional<RightOfWayPhaseRing> ring = phase_book_->FindPhaseRing(rule_id);
  if (ring.has_value()) {
    const optional<RightOfWayPhaseProvider::Result> phase_result
        = phase_provider_->GetPhase(ring->id());
    if (phase_result.has_value()) {
      const RightOfWayPhase::Id phase_id = phase_result->id;
      const RightOfWayPhase& phase = ring->phases().at(phase_id);
      const RightOfWayRule::State::Id state_id =
          phase.rule_states().at(rule_id);
      optional<Result::Next> next = nullopt;
      if (phase_result->next.has_value()) {
        const RightOfWayPhase::Id next_phase_id = phase_result->next->id;
        const RightOfWayPhase& next_phase = ring->phases().at(next_phase_id);
        const RightOfWayRule::State::Id next_state_id =
          next_phase.rule_states().at(rule_id);
        next = Result::Next{next_state_id, phase_result->next->duration_until};
      }
      return Result{state_id, next};
    }
  }
  return nullopt;
}

}  // namespace automotive
}  // namespace drake
