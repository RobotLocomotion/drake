#pragma once

#include <map>
#include <memory>
#include <utility>

#include <Eigen/Geometry>

#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/road_odometry.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// ClosestPose bundles together the RoadOdometry of a particular target along
/// with its distance measure relative to the ego vehicle.  Its intended use is
/// as the return argument for PoseSelector member functions.
template <typename T>
struct ClosestPose {
 public:
  /// Default constructor.
  ClosestPose() = default;

  /// Constructs the ClosestPose via a full parameterization.
  ClosestPose(const RoadOdometry<T>& odom, const T& dist)
      : odometry(odom), distance(dist) {}

  RoadOdometry<T> odometry{};
  T distance{0.};
};

/// Specifies whether to assess the cars ahead or behind the ego car at its
/// current orientation with respect to its lane.
enum class AheadOrBehind { kAhead = 0, kBehind = 1 };

/// Specifies whether to check ongoing lanes or both ongoing lanes and confluent
/// branches for traffic.
enum class ScanStrategy { kBranches, kPath };

/// If kCache, configures a planning system (e.g. IdmController, MobilPlanner)
/// to declare an abstract state that caches the last-computed RoadPosition.  If
/// kExhaustiveSearch, then the system will contain no abstract states.  Note
/// that the kCache option is for performance speedup (at the expense of
/// optimizer compatibility) by preventing a potentially sizeable computation
/// within RoadGeometry::ToRoadPosition().
enum class RoadPositionStrategy { kCache, kExhaustiveSearch };

/// PoseSelector is a class that provides the relevant pose or poses with
/// respect to a given ego vehicle driving within a given maliput road geometry.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
///
/// TODO(jadecastro): Enable AutoDiffXd support, and add unit tests.
template <typename T>
class PoseSelector {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseSelector)

  PoseSelector() = delete;

  /// Returns the leading and trailing vehicles in a given @p lane or @p road
  /// that are closest to an ego vehicle along its path, as measured along the
  /// `s`-coordinate of the ego vehicle's lane.  If @p path_or_branches is
  /// ScanStrategy::kPath, only poses in the same path as the ego (ahead or
  /// behind) are tracked; if ScanStrategy::kBranches, poses in lanes that
  /// eventually merge (converge to the same branch point that is on the default
  /// path of @p lane) are considered in addition to those along the ego car's
  /// default path.
  ///
  /// Users should take heed to the fact that ScanStrategy::kBranches does _not_
  /// assess the relationship between traffic and ego vehicle velocity when
  /// selecting poses.  Thus, cars within the same lane as the ego but with
  /// negative net-velocity (approaching the ego car, from the ego's
  /// point-of-view) could be ignored in favor of a car in a branch with
  /// positive net-velocity.
  ///
  /// The ego vehicle must be within the `driveable_bounds` of @p lane (i.e. the
  /// road is contiguous with @p lane along the `r`-direction).  This function
  /// is used, for instance, as logic for lane-change planners (e.g. MOBIL).
  /// The ego car's pose (@p ego_pose) and the poses of the traffic cars (@p
  /// traffic_poses) are provided.  The parameter @p scan_distance determines
  /// the distance along the sequence of lanes to scan before declaring that no
  /// traffic car is ahead (resp. behind) the ego car.
  ///
  /// @return A map of AheadOrBehind values to vehicle ClosestPoses (containing
  /// RoadOdometries and closest relative distances).  Relative distances are
  /// always positive, and a distance of positive infinity is returned if no
  /// traffic cars are found.  Note that when no vehicle is detected in front of
  /// (resp. behind) the ego vehicle, the respective RoadPosition within
  /// ClosestPoses will contain an `s`-value of positive (resp. negative)
  /// infinity.  Any traffic poses that are redunant with `ego_pose` (i.e. have
  /// the same RoadPosition as the ego car and thus the same `s` and `r` value)
  /// are discarded.  If no leading/trailing vehicles are seen within
  /// scan-distance of the ego car, `s`-positions are taken to be at infinite
  /// distances away from the ego car.  Note also that traffic vehicles having
  /// exactly the same `s`-position as the ego vehicle but with different
  /// `r`-value or are directly perpendicular in a Lane::to_left(),
  /// Lane::to_right() lane are taken to be *behind* the ego vehicle.  Note that
  /// this implementation is greatly simplified by considering poses as points
  /// (i.e. vehicle geometries are ignored).
  ///
  /// The RoadGeometry from which @p lane is drawn is required to have default
  /// branches set for all branches in the road network.
  static std::map<AheadOrBehind, const ClosestPose<T>> FindClosestPair(
      const maliput::api::Lane* lane,
      const systems::rendering::PoseVector<T>& ego_pose,
      const systems::rendering::PoseBundle<T>& traffic_poses,
      const T& scan_distance, ScanStrategy path_or_branches);

  /// Same as PoseSelector::FindClosestPair() except that it returns a single
  /// ClosestPose for either the vehicle ahead (AheadOrBehind::kAhead) or behind
  /// (AheadOrBehind::kBehind).
  ///
  /// Cars in other lanes are only tracked if they are in confluent lanes to a
  /// given branch point within the `scan_distance`; i.e. cars in two
  /// side-by-side lanes that never enter a branch point will not be selected.
  /// This assumes that the ego car has knowledge of the traffic cars' default
  /// lane.
  ///
  /// Note that when no car is detected in front of the ego car, the returned
  /// RoadOdometry within ClosestPose will contain an `s`-value of
  /// `std::numeric_limits<double>::infinity()`.
  static ClosestPose<T> FindSingleClosestPose(
      const maliput::api::Lane* lane,
      const systems::rendering::PoseVector<T>& ego_pose,
      const systems::rendering::PoseBundle<T>& traffic_poses,
      const T& scan_distance, const AheadOrBehind side,
      ScanStrategy path_or_branches);

  /// Extracts the vehicle's `s`-direction velocity based on its RoadOdometry @p
  /// road_odometry in the Lane coordinate frame.  Assumes the road has zero
  /// elevation and superelevation.  Throws if any element of
  /// `road_odometry.pos` is not within the respective bounds of
  /// `road_odometry.lane`.
  ///
  /// N.B. This function currently only provides exact derivatives for velocity
  /// in the `s` direction when the road is straight (no yaw angle variations).
  //
  // TODO(jadecastro) Enable AutoDiffXd for
  // maliput::api::Lane::GetOrientation().
  static T GetSigmaVelocity(const RoadOdometry<T>& road_odometry);
};

}  // namespace automotive
}  // namespace drake
