#include "drake/automotive/maliput/api/road_network.h"

#include <utility>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/rules/regions.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace api {

RoadNetwork::RoadNetwork(
    std::unique_ptr<const RoadGeometry> road_geometry,
    std::unique_ptr<const rules::RoadRulebook> rulebook,
    std::unique_ptr<const rules::TrafficLightBook> traffic_light_book,
    std::unique_ptr<IntersectionBook> intersection_book,
    std::unique_ptr<rules::PhaseRingBook> phase_ring_book,
    std::unique_ptr<rules::RuleStateProvider> rule_state_provider,
    std::unique_ptr<rules::PhaseProvider> phase_provider)
    : road_geometry_(std::move(road_geometry)),
      rulebook_(std::move(rulebook)),
      traffic_light_book_(std::move(traffic_light_book)),
      intersection_book_(std::move(intersection_book)),
      phase_ring_book_(std::move(phase_ring_book)),
      rule_state_provider_(std::move(rule_state_provider)),
      phase_provider_(std::move(phase_provider)) {
  DRAKE_THROW_UNLESS(road_geometry_.get() != nullptr);
  DRAKE_THROW_UNLESS(rulebook_.get() != nullptr);
  DRAKE_THROW_UNLESS(traffic_light_book_.get() != nullptr);
  DRAKE_THROW_UNLESS(intersection_book_.get() != nullptr);
  DRAKE_THROW_UNLESS(phase_ring_book_.get() != nullptr);
  DRAKE_THROW_UNLESS(rule_state_provider_.get() != nullptr);
  DRAKE_THROW_UNLESS(phase_provider_.get() != nullptr);

  // Confirms full DirectionUsageRule coverage. Currently, this is determined by
  // verifying that each Lane within the RoadGeometry has an associated
  // DirectionUsageRule. In the future, this check could be made even more
  // rigorous by confirming that the union of all DirectionUsageRule zones
  // covers the whole RoadGeometry.
  const auto& lanes_map = road_geometry_->ById().GetLanes();
  for (const auto& lane_map : lanes_map) {
    const LaneId lane_id = lane_map.first;
    const auto results =
        rulebook_->FindRules({{lane_id, {0.0, lane_map.second->length()}}}, 0);
    DRAKE_THROW_UNLESS(results.direction_usage.size() > 0);
  }
}

}  // namespace api
}  // namespace maliput
}  // namespace drake
