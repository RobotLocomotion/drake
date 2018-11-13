#include "drake/automotive/maliput/simple_phase_provider/simple_right_of_way_phase_provider.h"

#include <string>

#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace simple_phase_provider {

using api::rules::RightOfWayPhaseProvider;

void SimpleRightOfWayPhaseProvider::AddPhaseRing(
    const api::rules::RightOfWayPhaseRing::Id& id,
    const api::rules::RightOfWayPhase::Id& initial_phase) {
  auto result = phases_.emplace(id, initial_phase);
  if (!result.second) {
     throw std::logic_error(
        "Attempted to add multiple phase rings with id " + id.string());
  }
}


void SimpleRightOfWayPhaseProvider::SetPhase(
    const api::rules::RightOfWayPhaseRing::Id& id,
    const api::rules::RightOfWayPhase::Id& phase) {
  phases_.at(id) = phase;
}

optional<RightOfWayPhaseProvider::Result>
SimpleRightOfWayPhaseProvider::DoGetPhase(
    const api::rules::RightOfWayPhaseRing::Id& id) const {
  auto it = phases_.find(id);
  if (it == phases_.end()) {
    return nullopt;
  }
  // TODO(liang.fok) Add support for "next phase" and "duration until", #9993.
  return Result{it->second, nullopt};
}

}  // namespace simple_phase_provider
}  // namespace maliput
}  // namespace drake
