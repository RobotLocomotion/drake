#include "drake/automotive/maliput/api/rules/right_of_way_rule_phase_provider.h"

#include <utility>

#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

void RightOfWayRulePhaseProvider::AddPhase(const RightOfWayRulePhase& phase) {
  DRAKE_THROW_UNLESS(phases_.find(phase.id) == phases_.end());
  phases_.insert({phase.id, phase});
}

const optional<const RightOfWayRulePhase&>
RightOfWayRulePhaseProvider::GetPhase(const RightOfWayRulePhase::Id& id) const {
  if (phases_.find(id) == phases_.end()) {
    return nullopt;
  }
  return phases_.at(id);
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
