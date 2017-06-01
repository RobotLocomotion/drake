#include "drake/automotive/pose_selector.h"

#include <limits>
#include <memory>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/math/saturate.h"

namespace drake {
namespace automotive {

using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

template <typename T>
std::map<AheadOrBehind, const ClosestPose<T>> PoseSelector<T>::FindClosestPair(
    const Lane* lane, const PoseVector<T>& ego_pose,
    const PoseBundle<T>& traffic_poses, const T& scan_distance) {
  std::map<AheadOrBehind, const ClosestPose<T>> result;
  for (auto side : {AheadOrBehind::kAhead, AheadOrBehind::kBehind}) {
    result.insert(std::make_pair(
        side, FindSingleClosestPose(lane, ego_pose, traffic_poses,
                                    scan_distance, side)));
  }
  return result;
}

template <typename T>
ClosestPose<T> PoseSelector<T>::FindSingleClosestPose(
    const Lane* lane, const PoseVector<T>& ego_pose,
    const PoseBundle<T>& traffic_poses, const T& scan_distance,
    const AheadOrBehind side) {
  using std::abs;

  DRAKE_DEMAND(lane != nullptr);

  ClosestPose<T> result;
  result.odometry = make_infinite_odometry(lane, side);
  result.distance = std::numeric_limits<T>::infinity();
  ClosestPose<T> default_result = result;

  const GeoPosition ego_geo_position(ego_pose.get_isometry().translation());
  const LanePosition ego_lane_position =
      lane->ToLanePosition(ego_geo_position, nullptr, nullptr);
  LaneDirection lane_direction = CalcLaneDirection(
      {lane, ego_lane_position}, ego_pose.get_rotation(), side);
  const double ego_s = calc_lane_progress(lane_direction, ego_lane_position);
  T distance_scanned = T(-ego_s);

  // Traverse forward or backward from the current lane the given scan_distance,
  // looking for traffic cars.
  while (distance_scanned < scan_distance) {
    T distance_increment{0.};
    for (int i = 0; i < traffic_poses.get_num_poses(); ++i) {
      Isometry3<T> traffic_iso = traffic_poses.get_pose(i);
      const GeoPosition traffic_geo_position(traffic_iso.translation());

      if (ego_geo_position == traffic_geo_position) continue;
      if (!IsWithinLane(traffic_geo_position, lane_direction.lane)) continue;

      const LanePosition traffic_lane_position =
          lane_direction.lane->ToLanePosition(traffic_geo_position, nullptr,
                                              nullptr);
      const double traffic_s =
          calc_lane_progress(lane_direction, traffic_lane_position);

      const double s_delta = traffic_s - ego_s;
      // Ignore traffic that are not in the desired direction (ahead or behind)
      // of the ego car (with respect to the car's current direction).
      // Note that this check is only needed when the two share the same lane.
      if (distance_scanned <= T(0.)) {
        if (s_delta < 0.) continue;
        if (side == AheadOrBehind::kAhead && s_delta == 0.) continue;
      }

      // Ignore positions at the desired direction (ahead or behind) of the ego
      // car that are not closer than any other found so far.
      const double s_improvement =
          (side == AheadOrBehind::kAhead)
              ? result.odometry.pos.s() - traffic_lane_position.s()
              : traffic_lane_position.s() - result.odometry.pos.s();
      if (s_improvement < 0.) continue;

      // Update the result and incremental distance with the new candidate.
      result.odometry =
          RoadOdometry<T>({lane_direction.lane, traffic_lane_position},
                          traffic_poses.get_velocity(i));
      distance_increment = traffic_s;
    }

    if (abs(result.odometry.pos.s()) <
        std::numeric_limits<double>::infinity()) {
      // Figure out whether or not the result is within scan_distance.
      if (distance_scanned + distance_increment < scan_distance) {
        result.distance = distance_scanned + distance_increment;
        return result;
      }
    }
    // Increment distance_scanned.
    distance_scanned += T(lane_direction.lane->length());

    // Obtain the next lane_direction in the scanned sequence.
    GetDefaultOngoingLane(&lane_direction);
    if (lane_direction.lane == nullptr) return result;  // +/- Infinity.
  }
  return default_result;
}

template <typename T>
T PoseSelector<T>::GetSigmaVelocity(const RoadOdometry<T>& road_odometry) {
  const LanePosition sat_position{
      math::saturate(road_odometry.pos.s(), 0., road_odometry.lane->length()),
      road_odometry.pos.r(), road_odometry.pos.h()};
  const maliput::api::Rotation rot =
      road_odometry.lane->GetOrientation(sat_position);
  const Vector3<T>& vel = road_odometry.vel.get_velocity().translational();
  return vel(0) * std::cos(rot.yaw()) + vel(1) * std::sin(rot.yaw());
}

template <typename T>
bool PoseSelector<T>::IsWithinLane(const GeoPosition& geo_position,
                                   const Lane* lane) {
  double distance{};
  const LanePosition pos =
      lane->ToLanePosition(geo_position, nullptr, &distance);
  const maliput::api::RBounds r_bounds = lane->lane_bounds(pos.s());
  return (distance == 0. && pos.r() >= r_bounds.r_min &&
          pos.r() <= r_bounds.r_max);
}

template <typename T>
std::unique_ptr<LaneEnd> PoseSelector<T>::GetDefaultOngoingLane(
    LaneDirection* lane_direction) {
  const Lane* lane{lane_direction->lane};
  const bool with_s{lane_direction->with_s};
  std::unique_ptr<LaneEnd> branch =
      (with_s) ? lane->GetDefaultBranch(LaneEnd::kFinish)
               : lane->GetDefaultBranch(LaneEnd::kStart);
  if (branch == nullptr) {
    lane_direction->lane = nullptr;
    lane_direction->with_s = true;
    return branch;
  }
  lane_direction->lane = branch->lane;
  lane_direction->with_s = (branch->end == LaneEnd::kStart) ? true : false;
  return branch;
}

template <typename T>
RoadOdometry<T> PoseSelector<T>::make_infinite_odometry(
    const Lane* lane, const AheadOrBehind side) {
  const double infinite_distance =
      (side == AheadOrBehind::kAhead)
          ? std::numeric_limits<double>::infinity()
          : -std::numeric_limits<double>::infinity();
  const RoadPosition default_road_position(lane, {infinite_distance, 0., 0.});
  return {default_road_position, FrameVelocity<T>()};
}

template <typename T>
double PoseSelector<T>::calc_lane_progress(const LaneDirection& lane_direction,
                                           const LanePosition& lane_position) {
  const maliput::api::RBounds r_bounds =
      lane_direction.lane->driveable_bounds(lane_position.s());
  DRAKE_DEMAND(lane_position.s() >= 0. &&
               lane_position.s() <= lane_direction.lane->length() &&
               lane_position.r() >= r_bounds.r_min &&
               lane_position.r() <= r_bounds.r_max);
  const double new_s =
      (lane_direction.with_s)
          ? lane_position.s()
          : (lane_direction.lane->length() - lane_position.s());
  return new_s;
}

template <typename T>
LaneDirection PoseSelector<T>::CalcLaneDirection(
    const RoadPosition& road_position, const Eigen::Quaternion<T>& rotation,
    AheadOrBehind side) {
  // Get the vehicle's heading with respect to the trial lane.
  const Eigen::Quaternion<T> lane_rotation =
      road_position.lane->GetOrientation(road_position.pos).quat();
  const bool with_s = (side == AheadOrBehind::kAhead)
                          ? lane_rotation.dot(rotation) >= 0.0
                          : lane_rotation.dot(rotation) < 0.0;
  return LaneDirection(road_position.lane, with_s);
}

// These instantiations must match the API documentation in pose_selector.h.
template class PoseSelector<double>;

}  // namespace automotive
}  // namespace drake
