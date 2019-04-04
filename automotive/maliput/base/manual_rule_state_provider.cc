#include "drake/automotive/maliput/base/manual_rule_state_provider.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {

using api::rules::RightOfWayRule;

void ManualRuleStateProvider::AddState(
    const RightOfWayRule::Id& id,
    const RightOfWayRule::State::Id& initial_state) {
  auto result = states_.emplace(id, initial_state);
  if (!result.second) {
     throw std::logic_error(
        "Attempted to add multiple rules with id " + id.string());
  }
}

void ManualRuleStateProvider::SetState(
    const RightOfWayRule::Id& id,
    const RightOfWayRule::State::Id& state) {
  states_.at(id) = state;
}

drake::optional<api::rules::RuleStateProvider::RightOfWayResult>
ManualRuleStateProvider::DoGetState(
    const RightOfWayRule::Id& id) const {
  auto it = states_.find(id);
  if (it == states_.end()) {
    return nullopt;
  }
  return api::rules::RuleStateProvider::RightOfWayResult{it->second, nullopt};
}

}  // namespace maliput
}  // namespace drake
