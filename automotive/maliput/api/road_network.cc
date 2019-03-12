#include "drake/automotive/maliput/api/road_network.h"

#include <utility>

#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace api {

RoadNetwork::RoadNetwork(
    std::unique_ptr<const RoadGeometry> road_geometry,
    std::unique_ptr<const rules::RoadRulebook> rulebook,
    std::vector<std::unique_ptr<Intersection>> intersections,
    std::unique_ptr<rules::RightOfWayPhaseBook> phase_book,
    std::unique_ptr<rules::RightOfWayStateProvider> state_provider,
    std::unique_ptr<rules::RightOfWayPhaseProvider> phase_provider,
    std::vector<rules::SpeedLimitRule> speed_limits)
    : road_geometry_(std::move(road_geometry)),
      rulebook_(std::move(rulebook)),
      intersections_(std::move(intersections)),
      phase_book_(std::move(phase_book)),
      state_provider_(std::move(state_provider)),
      phase_provider_(std::move(phase_provider)),
      speed_limits_(std::move(speed_limits)) {
  DRAKE_THROW_UNLESS(road_geometry_.get() != nullptr);
  DRAKE_THROW_UNLESS(rulebook_.get() != nullptr);
  DRAKE_THROW_UNLESS(phase_book_.get() != nullptr);
  DRAKE_THROW_UNLESS(state_provider_.get() != nullptr);
  DRAKE_THROW_UNLESS(phase_provider_.get() != nullptr);
  for (int i = 0; i < static_cast<int>(intersections_.size()); ++i) {
    intersections_map_[intersections_.at(i)->id()] = intersections_.at(i).get();
  }
}

optional<const Intersection*> RoadNetwork::intersection(
    const Intersection::Id& id) const {
  if (intersections_map_.find(id) == intersections_map_.end()) {
    return nullopt;
  }
  return intersections_map_.at(id);
}

}  // namespace api
}  // namespace maliput
}  // namespace drake
