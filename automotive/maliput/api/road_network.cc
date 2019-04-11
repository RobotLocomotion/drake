#include "drake/automotive/maliput/api/road_network.h"

#include <utility>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace api {

RoadNetwork::RoadNetwork(
    std::unique_ptr<const RoadGeometry> road_geometry,
    std::unique_ptr<const rules::RoadRulebook> rulebook,
    std::unique_ptr<const rules::TrafficLightBook> traffic_light_book,
    std::vector<std::unique_ptr<Intersection>> intersections,
    std::unique_ptr<rules::PhaseRingBook> phase_ring_book,
    std::unique_ptr<rules::RuleStateProvider> rule_state_provider,
    std::unique_ptr<rules::PhaseProvider> phase_provider,
    std::vector<rules::SpeedLimitRule> speed_limits,
    std::vector<rules::DirectionUsageRule> direction_usage_rules)
    : road_geometry_(std::move(road_geometry)),
      rulebook_(std::move(rulebook)),
      traffic_light_book_(std::move(traffic_light_book)),
      intersections_(std::move(intersections)),
      phase_ring_book_(std::move(phase_ring_book)),
      rule_state_provider_(std::move(rule_state_provider)),
      phase_provider_(std::move(phase_provider)),
      speed_limits_(std::move(speed_limits)),
      direction_usage_rules_(std::move(direction_usage_rules)) {
  DRAKE_THROW_UNLESS(road_geometry_.get() != nullptr);
  DRAKE_THROW_UNLESS(rulebook_.get() != nullptr);
  DRAKE_THROW_UNLESS(traffic_light_book_.get() != nullptr);
  DRAKE_THROW_UNLESS(phase_ring_book_.get() != nullptr);
  DRAKE_THROW_UNLESS(rule_state_provider_.get() != nullptr);
  DRAKE_THROW_UNLESS(phase_provider_.get() != nullptr);
  for (int i = 0; i < static_cast<int>(intersections_.size()); ++i) {
    intersections_map_[intersections_.at(i)->id()] = intersections_.at(i).get();
  }

  // Confirms full DirectionUsageRule coverage. Currently, this is determined by
  // verifying that each Lane within the RoadGeometry has an associated
  // DirectionUsageRule. In the future, this check could be made even more
  // rigorous by confirming that the union of all DirectionUsageRule zones
  // covers the whole RoadGeometry.
  const auto& lanes_map = road_geometry_->ById().GetLanes();
  for (const auto& lane_map : lanes_map) {
    const LaneId lane_id = lane_map.first;
    const auto RuleIdMatchesLaneId = [&](rules::DirectionUsageRule& rule) {
      return rule.zone().lane_id() == lane_id;
    };
    const auto lane_rule = std::find_if(direction_usage_rules_.begin(),
        direction_usage_rules_.end(), RuleIdMatchesLaneId);
    DRAKE_THROW_UNLESS(lane_rule != direction_usage_rules_.end());
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
