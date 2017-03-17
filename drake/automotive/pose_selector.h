#pragma once

#include <utility>

#include <Eigen/Geometry>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {
namespace pose_selector {

/// Returns the leading and trailing cars that have closest `s`-coordinates in a
/// given @p traffic_lane to an ego car as if the ego car were traveling in @p
/// traffic_lane at its current `s`-position.  The ego car's pose @ego_pose and
/// the poses of the traffic cars are assumed to exist on the same @p road.  If
/// @p traffic_lane is `nullptr`, the the ego car's current lane is used.  If no
/// cars are seen within @p traffic_lane, car `s`-positions are taken to be at
/// infinite distances away from the ego car.
///
/// The return values are a pair of leading/trailing RoadPositions. Note that
/// when no car is detected in front of (resp. behind) the ego car, the
/// respective RoadPosition will contain an `s`-value of positive
/// (resp. negative) infinity (`std::numeric_limits<double>::infinity()`).
///
/// N.B. When comparing across lanes, it is assumed that @p road is configured
/// such that a comparison between the `s`-positions of any two cars on the road
/// is meaningful.  For instance, if car A is at `s = 10 m` in lane 0's frame
/// and car B is at `s = 0 m` in lane 1's frame then, if car A moved into lane
/// 1, it would be 10 meters ahead of car B.  Only straight multi-lane roads are
/// supported presently.
///
/// TODO(jadecastro): Support road networks containing multi-lane segments
/// (#4934).
const std::pair<maliput::api::RoadPosition, maliput::api::RoadPosition>
FindClosestPair(
    const maliput::api::RoadGeometry& road,
    const systems::rendering::PoseVector<double>& ego_pose,
    const systems::rendering::PoseBundle<double>& traffic_poses,
    const maliput::api::Lane* traffic_lane = nullptr);

/// Same as FindClosestPair() except that: (1) it only considers the ego car's
/// lane and (2) it returns a single the RoadPosition of the leading vehicle.
///
///  Note that when no car is detected in front of the ego car, the returned
///  RoadPosition will contain an `s`-value of
///  `std::numeric_limits<double>::infinity()`.
const maliput::api::RoadPosition FindClosestLeading(
    const maliput::api::RoadGeometry& road,
    const systems::rendering::PoseVector<double>& ego_pose,
    const systems::rendering::PoseBundle<double>& traffic_poses);

/// Computes the RoadPosition for a car whose @p pose is located on a given @p
/// road.
const maliput::api::RoadPosition CalcRoadPosition(
    const maliput::api::RoadGeometry& road, const Isometry3<double>& pose);

}  // namespace pose_selector
}  // namespace automotive
}  // namespace drake
