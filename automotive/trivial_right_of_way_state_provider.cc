#include "drake/automotive/trivial_right_of_way_state_provider.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_throw.h"

namespace drake {
namespace automotive {

using maliput::api::rules::RightOfWayRule;

void TrivialRightOfWayStateProvider::AddState(
    const RightOfWayRule::Id& id,
    const RightOfWayRule::DynamicState& initial_state) {
  auto result = dynamic_states_.emplace(id, initial_state);
  if (!result.second) {
     throw std::logic_error(
        "Attempted to add multiple rules with id " + id.string());
  }
}

void TrivialRightOfWayStateProvider::SetState(
    const RightOfWayRule::Id& id,
    const RightOfWayRule::DynamicState& state) {
  dynamic_states_.at(id) = state;
}

RightOfWayRule::DynamicState TrivialRightOfWayStateProvider::DoGetState(
    const RightOfWayRule::Id& id) const {
  return dynamic_states_.at(id);
}

}  // namespace automotive
}  // namespace drake
