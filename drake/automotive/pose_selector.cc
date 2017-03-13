#include "drake/automotive/pose_selector.h"

#include <cmath>

#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

template <typename T>
const std::pair<RoadPosition, RoadPosition>
PoseSelector<T>::SelectClosestPositions(const RoadGeometry& road,
                                        const PoseVector<T>& ego_pose,
                                        const PoseBundle<T>& agent_poses,
                                        const Lane* agent_lane) {
  const RoadPosition& ego_position =
      GetRoadPosition(road, ego_pose.get_isometry());
  DRAKE_DEMAND(ego_position.lane != nullptr);
  // Take the ego car's lane by default.
  const Lane* const lane =
      (agent_lane == nullptr) ? ego_position.lane : agent_lane;

  // Reconstruct leading and trailing vehicle positions as if they are
  RoadPosition result_leading = RoadPosition(lane, LanePosition(1e6, 0, 0));
  RoadPosition result_trailing = RoadPosition(lane, LanePosition(-1e6, 0, 0));
  for (int i = 0; i < agent_poses.get_num_poses(); ++i) {
    const RoadPosition& agent_position =
        GetRoadPosition(road, agent_poses.get_pose(i));
    const T& s_agent = agent_position.pos.s;

    // If this pose is not the ego car and is in the correct lane, then plop it
    // into the correct bin.
    if (ego_position.lane->id().id != lane->id().id ||
        s_agent != ego_position.pos.s) {
      if (agent_position.lane->id().id == lane->id().id &&
          result_trailing.pos.s < s_agent && s_agent < result_leading.pos.s) {
        if (s_agent >= ego_position.pos.s)
          result_leading = agent_position;
        else
          result_trailing = agent_position;
      }
    }
  }
  return std::make_pair(result_leading, result_trailing);
}

template <typename T>
const RoadPosition PoseSelector<T>::SelectClosestLeadingPosition(
    const RoadGeometry& road, const PoseVector<T>& ego_pose,
    const PoseBundle<T>& agent_poses) {
  return SelectClosestPositions(road, ego_pose, agent_poses).first;
}

template <typename T>
const RoadPosition PoseSelector<T>::GetRoadPosition(const RoadGeometry& road,
                                                    const Isometry3<T>& pose) {
  const maliput::api::GeoPosition& geo_position = maliput::api::GeoPosition(
      pose.translation().x(), pose.translation().y(), pose.translation().z());
  return road.ToRoadPosition(geo_position, nullptr, nullptr, nullptr);
}

// These instantiations must match the API documentation in pose_selector.h.
template class PoseSelector<double>;

}  // namespace automotive
}  // namespace drake
