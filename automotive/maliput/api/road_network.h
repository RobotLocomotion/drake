#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "drake/automotive/maliput/api/intersection.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/rules/direction_usage_rule.h"
#include "drake/automotive/maliput/api/rules/phase_provider.h"
#include "drake/automotive/maliput/api/rules/phase_ring_book.h"
#include "drake/automotive/maliput/api/rules/road_rulebook.h"
#include "drake/automotive/maliput/api/rules/rule_state_provider.h"
#include "drake/automotive/maliput/api/rules/speed_limit_rule.h"
#include "drake/automotive/maliput/api/rules/traffic_light_book.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"

namespace drake {
namespace maliput {
namespace api {

/// A container that aggregates everything pertaining to Maliput.
class RoadNetwork {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RoadNetwork)

  /// Constructs an RoadNetwork instance. All parameters are required.
  /// @p direction_usage_rules must cover the whole @p road_geometry.
  RoadNetwork(std::unique_ptr<const RoadGeometry> road_geometry,
              std::unique_ptr<const rules::RoadRulebook> rulebook,
              std::unique_ptr<const rules::TrafficLightBook> traffic_light_book,
              std::vector<std::unique_ptr<Intersection>> intersections,
              std::unique_ptr<rules::PhaseRingBook> phase_ring_book,
              std::unique_ptr<rules::RuleStateProvider> rule_state_provider,
              std::unique_ptr<rules::PhaseProvider> phase_provider,
              std::vector<rules::SpeedLimitRule> speed_limits,
              std::vector<rules::DirectionUsageRule> direction_usage_rules);

  virtual ~RoadNetwork() = default;

  const RoadGeometry* road_geometry() const { return road_geometry_.get(); }

  const rules::RoadRulebook* rulebook() const { return rulebook_.get(); }

  const rules::TrafficLightBook* traffic_light_book() const {
    return traffic_light_book_.get();
  }

  /// Returns a pointer to the Intersection with the specified ID, or nullopt if
  /// no such Intersection exists.
  optional<const Intersection*> intersection(const Intersection::Id& id) const;

  const rules::PhaseRingBook* phase_ring_book() const {
    return phase_ring_book_.get();
  }

  const rules::RuleStateProvider* rule_state_provider() const {
    return rule_state_provider_.get();
  }

  const rules::PhaseProvider* phase_provider() const {
    return phase_provider_.get();
  }

  const std::vector<rules::SpeedLimitRule>* speed_limits() const {
    return &speed_limits_;
  }

  const std::vector<rules::DirectionUsageRule>* direction_usage_rules() const {
    return &direction_usage_rules_;
  }

 private:
  std::unique_ptr<const RoadGeometry> road_geometry_;
  std::unique_ptr<const rules::RoadRulebook> rulebook_;
  std::unique_ptr<const rules::TrafficLightBook> traffic_light_book_;
  std::vector<std::unique_ptr<Intersection>> intersections_;
  std::unordered_map<Intersection::Id, Intersection*> intersections_map_;
  std::unique_ptr<rules::PhaseRingBook> phase_ring_book_;
  std::unique_ptr<rules::RuleStateProvider> rule_state_provider_;
  std::unique_ptr<rules::PhaseProvider> phase_provider_;
  std::vector<rules::SpeedLimitRule> speed_limits_;
  std::vector<rules::DirectionUsageRule> direction_usage_rules_;
};

}  // namespace api
}  // namespace maliput
}  // namespace drake
