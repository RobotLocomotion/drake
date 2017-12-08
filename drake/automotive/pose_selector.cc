#include "drake/automotive/pose_selector.h"

#include <limits>
#include <memory>
#include <utility>

#include "drake/common/autodiffxd_make_coherent.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/extract_double.h"

namespace drake {
namespace automotive {

using maliput::api::GeoPositionT;
using maliput::api::Lane;
using maliput::api::LaneEnd;
using maliput::api::LanePosition;
using maliput::api::LanePositionT;
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

  const GeoPositionT<T> ego_geo_position =
      GeoPositionT<T>::FromXyz(ego_pose.get_isometry().translation());
  const LanePositionT<T> ego_lane_position =
      lane->ToLanePositionT<T>(ego_geo_position, nullptr, nullptr);
  LaneDirection lane_direction =
      CalcLaneDirection(lane, ego_lane_position, ego_pose.get_rotation(), side);

  ClosestPose<T> result;
  result.odometry = MakeInfiniteOdometry(lane_direction, ego_pose);
  result.distance = MakeInfiniteDistance(ego_pose);
  const ClosestPose<T> default_result = result;

  const T ego_s = CalcLaneProgress(lane_direction, ego_lane_position);
  T distance_scanned = T(-ego_s);  // N.B. ego_s is negated to recover the
                                   // remaining distance to the end of the lane
                                   // when `distance_scanned` is incremented by
                                   // the ego car's lane length.
  const bool ego_with_s = lane_direction.with_s;

  // Traverse forward or backward from the current lane the given scan_distance,
  // looking for traffic cars.
  while (distance_scanned < scan_distance) {
    T distance_increment{0.};
    for (int i = 0; i < traffic_poses.get_num_poses(); ++i) {
      const Isometry3<T> traffic_isometry = traffic_poses.get_pose(i);
      const GeoPositionT<T> traffic_geo_position =
          GeoPositionT<T>::FromXyz(traffic_isometry.translation());

      if (ego_geo_position == traffic_geo_position) continue;
      if (!IsWithinLane(traffic_geo_position, lane_direction.lane)) continue;

      const LanePositionT<T> traffic_lane_position =
          lane_direction.lane->ToLanePositionT<T>(traffic_geo_position, nullptr,
                                                  nullptr);
      const T traffic_s =
          CalcLaneProgress(lane_direction, traffic_lane_position);

      const T s_delta = traffic_s - ego_s;
      // Ignore traffic cars that are not in the desired direction (ahead or
      // behind) of the ego car (with respect to the car's current direction).
      // Cars with identical s-values as the ego but shifted laterally are
      // treated as `kBehind` cars.  Note that this check is only needed when
      // the two share the same lane or, equivalently, `distance_scanned <= 0`.
      if (distance_scanned <= T(0.)) {
        if (s_delta < 0.) continue;
        if (side == AheadOrBehind::kAhead && s_delta == 0.) continue;
      }

      // Ignore positions at the desired direction (ahead or behind) of the ego
      // car that are not closer than any other found so far.
      const T s_solution_difference =
          result.odometry.pos.s() - traffic_lane_position.s();
      const T s_improvement =
          (ego_with_s) ? s_solution_difference : -s_solution_difference;
      if (s_improvement < 0.) continue;

      // Update the result and incremental distance with the new candidate.
      result.odometry =
          RoadOdometry<T>(lane_direction.lane, traffic_lane_position,
                          traffic_poses.get_velocity(i));
      distance_increment = traffic_s;
    }

    if (abs(result.odometry.pos.s()) < std::numeric_limits<T>::infinity()) {
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
    if (lane_direction.lane == nullptr) {
      return result;
    }
  }
  return default_result;
}

template <typename T>
T PoseSelector<T>::GetSigmaVelocity(const RoadOdometry<T>& road_odometry) {
  DRAKE_DEMAND(IsWithinLane(road_odometry.pos, road_odometry.lane));
  const LanePosition& lane_pos =
      LanePosition(ExtractDoubleOrThrow(road_odometry.pos.s()),
                   ExtractDoubleOrThrow(road_odometry.pos.r()),
                   ExtractDoubleOrThrow(road_odometry.pos.h()));
  const maliput::api::Rotation rot =
      road_odometry.lane->GetOrientation(lane_pos);
  multibody::SpatialVelocity<T> road_odometry_velocity =
      road_odometry.vel.get_velocity();
  const Vector3<T>& vel = road_odometry_velocity.translational();
  return vel(0) * std::cos(rot.yaw()) + vel(1) * std::sin(rot.yaw());
}

template <typename T>
bool PoseSelector<T>::IsWithinDriveable(const LanePositionT<T>& lane_position,
                                        const Lane* lane) {
  if (lane_position.s() < 0. || lane_position.s() > lane->length()) {
    return false;
  }
  const maliput::api::RBounds r_bounds =
      lane->driveable_bounds(ExtractDoubleOrThrow(lane_position.s()));
  if (lane_position.r() < r_bounds.min() ||
      lane_position.r() > r_bounds.max()) {
    return false;
  }
  const maliput::api::HBounds h_bounds =
      lane->elevation_bounds(ExtractDoubleOrThrow(lane_position.s()),
                             ExtractDoubleOrThrow(lane_position.r()));
  return (lane_position.h() >= h_bounds.min() &&
          lane_position.h() <= h_bounds.max());
}

template <typename T>
bool PoseSelector<T>::IsWithinLane(const GeoPositionT<T>& geo_position,
                                   const Lane* lane) {
  T distance{};
  const LanePositionT<T> pos =
      lane->ToLanePositionT<T>(geo_position, nullptr, &distance);
  const maliput::api::RBounds r_bounds =
      lane->lane_bounds(ExtractDoubleOrThrow(pos.s()));
  return (distance == 0. && pos.r() >= r_bounds.min() &&
          pos.r() <= r_bounds.max());
}

template <typename T>
bool PoseSelector<T>::IsWithinLane(const LanePositionT<T>& lane_position,
                                   const Lane* lane) {
  if (IsWithinDriveable(lane_position, lane)) {
    const maliput::api::RBounds r_bounds =
        lane->lane_bounds(ExtractDoubleOrThrow(lane_position.s()));
    if (lane_position.r() >= r_bounds.min() ||
        lane_position.r() <= r_bounds.max()) {
      return true;
    }
  }
  return false;
}

template <typename T>
optional<LaneEnd> PoseSelector<T>::GetDefaultOngoingLane(
    LaneDirection* lane_direction) {
  const Lane* const lane{lane_direction->lane};
  const bool with_s{lane_direction->with_s};
  optional<LaneEnd> branch =
      (with_s) ? lane->GetDefaultBranch(LaneEnd::kFinish)
               : lane->GetDefaultBranch(LaneEnd::kStart);
  if (!branch) {
    lane_direction->lane = nullptr;
    lane_direction->with_s = true;
    return branch;
  }
  lane_direction->lane = branch->lane;
  lane_direction->with_s = (branch->end == LaneEnd::kStart) ? true : false;
  return branch;
}

template <typename T>
RoadOdometry<T> PoseSelector<T>::MakeInfiniteOdometry(
    const LaneDirection& lane_direction, const PoseVector<T>& ego_pose) {
  T infinite_position = (lane_direction.with_s)
                            ? std::numeric_limits<T>::infinity()
                            : -std::numeric_limits<T>::infinity();
  T zero(0.);
  autodiffxd_make_coherent(ego_pose.get_isometry().translation().x(), &zero);
  autodiffxd_make_coherent(ego_pose.get_isometry().translation().x(),
                           &infinite_position);
  const LanePositionT<T> lane_position(infinite_position, zero, zero);
  FrameVelocity<T> frame_velocity;
  auto velocity = frame_velocity.get_mutable_value();
  for (int i{0}; i < frame_velocity.kSize; ++i) {
    autodiffxd_make_coherent(ego_pose.get_isometry().translation().x(),
                             &velocity(i));
  }
  // TODO(jadecastro) Consider moving the above autodiffxd_make_coherent() step
  // to BasicVector().
  return {lane_direction.lane, lane_position, frame_velocity};
}

template <typename T>
T PoseSelector<T>::MakeInfiniteDistance(const PoseVector<T>& ego_pose) {
  T infinite_distance = std::numeric_limits<T>::infinity();
  autodiffxd_make_coherent(ego_pose.get_isometry().translation().x(),
                           &infinite_distance);
  return infinite_distance;
}

template <typename T>
T PoseSelector<T>::CalcLaneProgress(const LaneDirection& lane_direction,
                                    const LanePositionT<T>& lane_position) {
  DRAKE_DEMAND(IsWithinDriveable(lane_position, lane_direction.lane));
  if (lane_direction.with_s) {
    return lane_position.s();
  } else {
    return T(lane_direction.lane->length()) - lane_position.s();
  }
}

template <typename T>
LaneDirection PoseSelector<T>::CalcLaneDirection(
    const Lane* lane, const LanePositionT<T>& lane_position,
    const Eigen::Quaternion<T>& rotation, AheadOrBehind side) {
  // Get the vehicle's heading with respect to the current lane; use it to
  // determine if the vehicle is facing with or against the lane's canonical
  // direction.
  const LanePosition lane_pos =
      LanePosition(ExtractDoubleOrThrow(lane_position.s()),
                   ExtractDoubleOrThrow(lane_position.r()),
                   ExtractDoubleOrThrow(lane_position.h()));
  const Eigen::Quaternion<T> lane_rotation =
      lane->GetOrientation(lane_pos).quat();
  // The dot product of two quaternions is the cosine of half the angle between
  // the two rotations.  Given two quaternions q₀, q₁ and letting θ be the angle
  // difference between them, then -π/2 ≤ θ ≤ π/2 iff q₀.q₁ ≥ √2/2.
  const bool with_s = (side == AheadOrBehind::kAhead)
                          ? lane_rotation.dot(rotation) >= sqrt(2.) / 2.
                          : lane_rotation.dot(rotation) < sqrt(2.) / 2.;
  return LaneDirection(lane, with_s);
}

}  // namespace automotive
}  // namespace drake

// These instantiations must match the API documentation in pose_selector.h.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::automotive::PoseSelector)
