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

/// PoseSelector contains utilities for selecting among several agent cars those
/// that have the closest relative `s`-positions to an ego car traveling in the
/// same maliput lane or an adjacent one.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_systems
template <typename T>
class PoseSelector {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseSelector)
  PoseSelector() = delete;

  /// Compares the Lane-space poses collected within @p agent_poses against an
  /// @p ego_pose and returns a pair of the closest {leading, trailing}
  /// RoadPositions.  All cars are assumed to be traveling on the same @p road.
  /// Regardless of the ego car's lane, a comparison is made for cars in @p
  /// agent_lane.  If @p agent_lane is `nullptr`, the the ego car's current lane
  /// is used.  If no cars are seen within @p agent_lane, the cars are taken to
  /// be at infinite distances away from the ego car.
  ///
  /// N.B. Assumes that @p road is set up such that a comparison between the
  /// `s`-positions of any two cars on the road is meaningful.
  static const std::pair<maliput::api::RoadPosition, maliput::api::RoadPosition>
  SelectClosestPositions(const maliput::api::RoadGeometry& road,
                         const systems::rendering::PoseVector<T>& ego_pose,
                         const systems::rendering::PoseBundle<T>& agent_poses,
                         const maliput::api::Lane* agent_lane = nullptr);

  /// Compares the Lane-space poses collected within @p agent_poses against an
  /// @p ego_pose and returns the closest leading RoadPosition for agent cars
  /// traveling in the same lane as the ego car.  All cars are assumed to be
  /// traveling on the same @p road.  If no cars are seen within @p agent_lane,
  /// the leading car is taken to be at an infinite distance away from the ego
  /// car.
  static const maliput::api::RoadPosition SelectClosestLeadingPosition(
      const maliput::api::RoadGeometry& road,
      const systems::rendering::PoseVector<T>& ego_pose,
      const systems::rendering::PoseBundle<T>& agent_poses);

  /// Computes the RoadPosition for a car whose @p pose is located on a given @p
  /// road.
  static const maliput::api::RoadPosition CalcRoadPosition(
      const maliput::api::RoadGeometry& road, const Isometry3<T>& pose);
};

}  // namespace automotive
}  // namespace drake
