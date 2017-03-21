#include "drake/automotive/pose_selector.h"

#include <cmath>
#include <limits>

#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {
namespace pose_selector {

using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

const std::pair<RoadPosition, RoadPosition> FindClosestPair(
    const RoadGeometry& road, const PoseVector<double>& ego_pose,
    const PoseBundle<double>& traffic_poses, const Lane* traffic_lane) {
  const RoadPosition& ego_position =
      CalcRoadPosition(road, ego_pose.get_isometry());
  DRAKE_DEMAND(ego_position.lane != nullptr);
  // Take the ego car's lane by default.
  const Lane* const lane =
      (traffic_lane == nullptr) ? ego_position.lane : traffic_lane;

  // Default the leading and trailing positions to extend to, respectively,
  // positive and negative infinity.
  RoadPosition result_leading = RoadPosition(
      lane, LanePosition(std::numeric_limits<double>::infinity(), 0., 0.));
  RoadPosition result_trailing = RoadPosition(
      lane, LanePosition(-std::numeric_limits<double>::infinity(), 0., 0.));
  for (int i = 0; i < traffic_poses.get_num_poses(); ++i) {
    const RoadPosition& traffic_position =
        CalcRoadPosition(road, traffic_poses.get_pose(i));
    const double& s_traffic = traffic_position.pos.s;

    if (traffic_position.lane->id().id != lane->id().id) continue;

    // If this pose is not the ego car and it is in the correct lane, then
    // insert it into the correct "leading" or "trailing" bin.
    if (ego_position.lane->id().id != lane->id().id ||
        s_traffic != ego_position.pos.s) {
      if (result_trailing.pos.s < s_traffic &&
          s_traffic < result_leading.pos.s) {
        // N.B. The ego car and traffic may reside in different lanes.
        if (s_traffic >= ego_position.pos.s)
          result_leading = traffic_position;
        else
          result_trailing = traffic_position;
      }
    }
  }
  return std::make_pair(result_leading, result_trailing);
}

const RoadPosition FindClosestLeading(
    const RoadGeometry& road, const PoseVector<double>& ego_pose,
    const PoseBundle<double>& traffic_poses) {
  return FindClosestPair(road, ego_pose, traffic_poses).first;
}

const RoadPosition CalcRoadPosition(const RoadGeometry& road,
                                    const Isometry3<double>& pose) {
  return road.ToRoadPosition(
      maliput::api::GeoPosition(pose.translation().x(), pose.translation().y(),
                                pose.translation().z()),
      nullptr, nullptr, nullptr);
}

}  // namespace pose_selector
}  // namespace automotive
}  // namespace drake
