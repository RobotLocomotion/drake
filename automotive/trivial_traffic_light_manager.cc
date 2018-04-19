#include "drake/automotive/trivial_traffic_light_manager.h"

#include "drake/common/drake_throw.h"

namespace drake {
namespace automotive {

using maliput::api::rules::RightOfWayRule;

void TrivialTrafficLightManager::AddState(
    const RightOfWayRule::Id& id,
    const RightOfWayRule::DynamicState& initial_state) {
  DRAKE_THROW_UNLESS(dynamic_states_.count(id) == 0);
  dynamic_states_[id] = initial_state;
}

/// Sets the dynamic state of a RightOfWayRule within this manager.
///
/// Throws an exception if no dynamic state with @p id exists in this manager.
void TrivialTrafficLightManager::SetState(
    const RightOfWayRule::Id& id,
    const RightOfWayRule::DynamicState& state) {
  DRAKE_THROW_UNLESS(dynamic_states_.count(id) == 1);
  dynamic_states_[id] = state;
}

RightOfWayRule::DynamicState TrivialTrafficLightManager::DoGetState(
    const RightOfWayRule::Id& id) const {
  DRAKE_THROW_UNLESS(dynamic_states_.count(id) == 1);
  return dynamic_states_.at(id);
}

}  // namespace automotive
}  // namespace drake
