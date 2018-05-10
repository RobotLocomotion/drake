#include "drake/automotive/trivial_right_of_way_state_provider.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_throw.h"

namespace drake {
namespace automotive {

using maliput::api::rules::RightOfWayRule;

void TrivialRightOfWayStateProvider::AddState(
    const RightOfWayRule::Id& id,
    const RightOfWayRule::State::Id& initial_state) {
  auto result = states_.emplace(id, initial_state);
  if (!result.second) {
     throw std::logic_error(
        "Attempted to add multiple rules with id " + id.string());
  }
}

void TrivialRightOfWayStateProvider::SetState(
    const RightOfWayRule::Id& id,
    const RightOfWayRule::State::Id& state) {
  states_.at(id) = state;
}

drake::optional<maliput::api::rules::RightOfWayStateProvider::Result>
TrivialRightOfWayStateProvider::DoGetState(
    const RightOfWayRule::Id& id) const {
  auto it = states_.find(id);
  if (it == states_.end()) {
    return nullopt;
  }
  return Result{it->second, nullopt};
}

}  // namespace automotive
}  // namespace drake
