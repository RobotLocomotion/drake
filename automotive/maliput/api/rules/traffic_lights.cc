#include "drake/automotive/maliput/api/rules/traffic_lights.h"

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

Bulb::Bulb(const Bulb::Id& id, const GeoPosition& position_bulb_group,
           const Rotation& orientation_bulb_group, const BulbColor& color,
           const BulbType& type, const optional<double>& arrow_orientation_rad)
    : id_(id),
      position_bulb_group_(position_bulb_group),
      orientation_bulb_group_(orientation_bulb_group),
      color_(color),
      type_(type),
      arrow_orientation_rad_(arrow_orientation_rad) {
  DRAKE_THROW_UNLESS(type_ != BulbType::kArrow ||
                     arrow_orientation_rad_ != nullopt);
  if (type_ != BulbType::kArrow) {
    DRAKE_THROW_UNLESS(arrow_orientation_rad_ == nullopt);
  }
}

}  // namespace rules
}  // namespace api
}  // namespace maliput
}  // namespace drake
