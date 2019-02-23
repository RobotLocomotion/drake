#include "drake/automotive/maliput/api/rules/traffic_lights.h"

namespace drake {
namespace maliput {
namespace api {
namespace rules {

std::map<BulbColor, const char*> BulbColorMapper() {
  return {
      {BulbColor::kRed, "Red"},
      {BulbColor::kYellow, "Yellow"},
      {BulbColor::kGreen, "Green"},
  };
}

std::map<BulbType, const char*> BulbTypeMapper() {
  return {
      {BulbType::kRound, "Round"},
      {BulbType::kArrow, "Arrow"},
  };
}

Bulb::Bulb(const Bulb::Id& id, const GeoPosition& position_bulb_group,
           const Rotation& orientation_bulb_group, const BulbColor& color,
           const BulbType& type)
    : id_(id),
      position_bulb_group_(position_bulb_group),
      orientation_bulb_group_(orientation_bulb_group),
      color_(color),
      type_(type) {}

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
