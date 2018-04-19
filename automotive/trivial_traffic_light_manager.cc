#include "drake/automotive/trivial_traffic_light_manager.h"

#include "drake/common/drake_throw.h"

namespace drake {
namespace automotive {

using maliput::api::rules::RightOfWayRule;

void TrivialTrafficLightManager::AddState(
    const RightOfWayRule::Id& id,
    const RightOfWayRule::DynamicState& initial_state) {
  DRAKE_THROW_UNLESS(dynamic_states_.count(id) == 0);
  auto result = dynamic_states_.emplace(id, initial_state);
  DRAKE_THROW_UNLESS(result.second);
}

void TrivialTrafficLightManager::SetState(
    const RightOfWayRule::Id& id,
    const RightOfWayRule::DynamicState& state) {
  dynamic_states_.at(id) = state;
}

RightOfWayRule::DynamicState TrivialTrafficLightManager::DoGetState(
    const RightOfWayRule::Id& id) const {
  return dynamic_states_.at(id);
}

}  // namespace automotive
}  // namespace drake
