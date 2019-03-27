#include "drake/automotive/maliput/base/phase_based_right_of_way_state_provider.h"

#include "drake/automotive/maliput/api/rules/phase.h"
#include "drake/automotive/maliput/api/rules/phase_ring.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {

using api::rules::Phase;
using api::rules::PhaseRing;
using api::rules::RightOfWayPhaseBook;
using api::rules::RightOfWayPhaseProvider;
using api::rules::RightOfWayRule;
using api::rules::RightOfWayStateProvider;

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
  optional<PhaseRing> ring = phase_book_->FindPhaseRing(rule_id);
  if (ring.has_value()) {
    const optional<RightOfWayPhaseProvider::Result> phase_result
        = phase_provider_->GetPhase(ring->id());
    if (phase_result.has_value()) {
      const Phase::Id phase_id = phase_result->id;
      const Phase& phase = ring->phases().at(phase_id);
      const RightOfWayRule::State::Id state_id =
          phase.rule_states().at(rule_id);
      optional<Result::Next> next = nullopt;
      if (phase_result->next.has_value()) {
        const Phase::Id next_phase_id = phase_result->next->id;
        const Phase& next_phase = ring->phases().at(next_phase_id);
        const RightOfWayRule::State::Id next_state_id =
          next_phase.rule_states().at(rule_id);
        next = Result::Next{next_state_id, phase_result->next->duration_until};
      }
      return Result{state_id, next};
    }
  }
  return nullopt;
}

}  // namespace maliput
}  // namespace drake
