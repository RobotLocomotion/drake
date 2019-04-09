#include "drake/automotive/maliput/api/rules/traffic_lights.h"

#include <algorithm>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

std::unordered_map<BulbColor, const char*, DefaultHash> BulbColorMapper() {
  std::unordered_map<BulbColor, const char*, DefaultHash> result;
  result.emplace(BulbColor::kRed, "Red");
  result.emplace(BulbColor::kYellow, "Yellow");
  result.emplace(BulbColor::kGreen, "Green");
  return result;
}

std::unordered_map<BulbType, const char*, DefaultHash> BulbTypeMapper() {
  std::unordered_map<BulbType, const char*, DefaultHash> result;
  result.emplace(BulbType::kRound, "Round");
  result.emplace(BulbType::kArrow, "Arrow");
  return result;
}

std::unordered_map<BulbState, const char*, DefaultHash> BulbStateMapper() {
  std::unordered_map<BulbState, const char*, DefaultHash> result;
  result.emplace(BulbState::kOff, "Off");
  result.emplace(BulbState::kOn, "On");
  result.emplace(BulbState::kBlinking, "Blinking");
  return result;
}

Bulb::Bulb(const Bulb::Id& id, const GeoPosition& position_bulb_group,
           const Rotation& orientation_bulb_group, const BulbColor& color,
           const BulbType& type, const optional<double>& arrow_orientation_rad,
           const optional<std::vector<BulbState>>& states,
           BoundingBox bounding_box)
    : id_(id),
      position_bulb_group_(position_bulb_group),
      orientation_bulb_group_(orientation_bulb_group),
      color_(color),
      type_(type),
      arrow_orientation_rad_(arrow_orientation_rad),
      bounding_box_(std::move(bounding_box)) {
  DRAKE_THROW_UNLESS(type_ != BulbType::kArrow ||
                     arrow_orientation_rad_ != nullopt);
  if (type_ != BulbType::kArrow) {
    DRAKE_THROW_UNLESS(arrow_orientation_rad_ == nullopt);
  }
  if (states == nullopt || states->size() == 0) {
    states_ = {BulbState::kOff, BulbState::kOn};
  } else {
    states_ = *states;
  }
}

BulbState Bulb::GetDefaultState() const {
  for (const auto& bulb_state :
       {BulbState::kOff, BulbState::kBlinking, BulbState::kOn}) {
    if (IsValidState(bulb_state)) {
      return bulb_state;
    }
  }
  DRAKE_UNREACHABLE();
}

bool Bulb::IsValidState(const BulbState& bulb_state) const {
  return std::find(states_.begin(), states_.end(), bulb_state) != states_.end();
}

BulbGroup::BulbGroup(const BulbGroup::Id& id,
                     const GeoPosition& position_traffic_light,
                     const Rotation& orientation_traffic_light,
                     const std::vector<Bulb>& bulbs)
    : id_(id),
      position_traffic_light_(position_traffic_light),
      orientation_traffic_light_(orientation_traffic_light),
      bulbs_(bulbs) {
  DRAKE_THROW_UNLESS(bulbs_.size() > 0);
}

optional<Bulb> BulbGroup::GetBulb(const Bulb::Id& id) const {
  for (const auto& bulb : bulbs_) {
    if (bulb.id() == id) {
      return bulb;
    }
  }
  return nullopt;
}

TrafficLight::TrafficLight(const TrafficLight::Id& id,
                           const GeoPosition& position_road_network,
                           const Rotation& orientation_road_network,
                           const std::vector<BulbGroup>& bulb_groups)
    : id_(id),
      position_road_network_(position_road_network),
      orientation_road_network_(orientation_road_network),
      bulb_groups_(bulb_groups) {}

optional<BulbGroup> TrafficLight::GetBulbGroup(const BulbGroup::Id& id) const {
  for (const auto& bulb_group : bulb_groups_) {
    if (bulb_group.id() == id) {
      return bulb_group;
    }
  }
  return nullopt;
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
