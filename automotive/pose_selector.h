#pragma once

#include <map>
#include <memory>
#include <utility>
#include <vector>

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

static constexpr double kMaximumScanDistance{100.};
static constexpr double kEgoSigmaVelocity{1.};  // Assumed ego velocity, used in
                                                // determining how far ahead to
                                                // search for traffic cars.

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
  typedef typename std::pair<const T, const maliput::api::LaneEnd>
      LaneEndDistance;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseSelector)

  PoseSelector() = delete;

  /// Returns the leading and trailing vehicles in a given @p lane or @p road
  /// that are closest to an ego vehicle along its path, as measured along the
  /// `s`-coordinate of the ego vehicle's lane.
  ///
  /// If the first argument is a RoadPosition object (@p road), then poses whose
  /// (converge to the same branch point) are considered in addition to those
  /// along the ego car's path.  Otherwise, if the argument is a Lane* object
  /// (@p lane), only poses in the same path as the ego (ahead or behind) are
  /// tracked.  Cars in other lanes are only tracked if they are confluent lanes
  /// to a given branch point within the `scan_ahead_distance`; i.e. cars in two
  /// side-by-side lanes that never enter a branch point will not be selected.
  /// It also assumes that the ego car has knowledge of the traffic cars'
  /// default lane (a reasonable assumption, given a short enough horizon).
  ///
  /// The ego vehicle must be within the `driveable_bounds` of @p lane (i.e. the
  /// road is contiguous with @p lane along the `r`-direction).  This function
  /// is used, for instance, as logic for lane-change planners (e.g. MOBIL).
  /// The ego car's pose (@p ego_pose) and the poses of the traffic cars (@p
  /// traffic_poses) are provided.  The parameter @p scan_distance determines
  /// the distance along the sequence of lanes to scan before declaring that no
  /// traffic car is ahead (resp. behind) the ego car.  If no leading/trailing
  /// vehicles are seen within @p traffic_lane, `s`-positions are taken to be at
  /// infinite distances away from the ego car.  Traffic vehicles having exactly
  /// the same s-position as the ego vehicle but situated in a different
  /// (parallel) lane are taken to be behind the ego vehicle.
  ///
  /// @return A map of AheadOrBehind values to vehicle ClosestPoses (containing
  /// RoadOdometries and closest relative distances).  Relative distances are
  /// always positive, and a distance of positive infinity is returned if no
  /// traffic cars are found.  Note that when no vehicle is detected in front of
  /// (resp. behind) the ego vehicle, the respective RoadPosition within
  /// ClosestPoses will contain an `s`-value of positive (resp. negative)
  /// infinity.  Any traffic poses that are redunant with `ego_pose` (i.e. have
  /// the same RoadPosition as the ego car) are discarded.
  ///
  /// The RoadGeometry from which @p lane is drawn is required to have default
  /// branches set for all branches in the road network.
  static std::map<AheadOrBehind, const ClosestPose<T>> FindClosestPair(
      const maliput::api::RoadGeometry& road,
      const systems::rendering::PoseVector<T>& ego_pose,
      const systems::rendering::PoseBundle<T>& traffic_poses,
      const T& scan_distance);

  /// (See docstring for FindClosestPair).
  static std::map<AheadOrBehind, const ClosestPose<T>> FindClosestPair(
      const maliput::api::Lane* lane,
      const systems::rendering::PoseVector<T>& ego_pose,
      const systems::rendering::PoseBundle<T>& traffic_poses,
      const T& scan_distance);

  /// Same as PoseSelector::FindClosestPair() except that it returns a single
  /// ClosestPose for either the vehicle ahead (AheadOrBehind::kAhead) or behind
  /// (AheadOrBehind::kBehind).
  ///
  /// If the first argument is a RoadPosition object, then poses in lanes that
  /// eventually merge (converge to the same branch point) are considered in
  /// addition to those along the ego car's path.  Otherwise, if the argument is
  /// a Lane* object, only poses in the same path as the ego (ahead or behind)
  /// are tracked.
  ///
  /// Note that when no car is detected in front of the ego car, the returned
  /// RoadOdometry within ClosestPose will contain an `s`-value of
  /// `std::numeric_limits<double>::infinity()`.
  static ClosestPose<T> FindSingleClosestPose(
      const maliput::api::RoadGeometry& road,
      const systems::rendering::PoseVector<T>& ego_pose,
      const systems::rendering::PoseBundle<T>& traffic_poses,
      const T& scan_distance, const AheadOrBehind side);

  /// (See docstring for FindSingleClosestPose).
  static ClosestPose<T> FindSingleClosestPose(
      const maliput::api::Lane* lane,
      const systems::rendering::PoseVector<T>& ego_pose,
      const systems::rendering::PoseBundle<T>& traffic_poses,
      const T& scan_distance, const AheadOrBehind side);

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

  /// Returns `true` if and only if @p lane_position is within the longitudinal
  /// (s), driveable (r) and elevation (h) bounds of the specified @p lane
  /// (i.e. within `linear_tolerance()` of `lane->driveable_bounds()` and
  /// `lane->elevation_bounds()`).
  static bool IsWithinDriveable(
      const maliput::api::LanePositionT<T>& lane_position,
      const maliput::api::Lane* lane);

  /// Returns `true` if and only if @p geo_position is within the longitudinal
  /// (s), lateral (r) and elevation (h) bounds of the specified @p lane
  /// (i.e. within `linear_tolerance()` of `lane->lane_bounds()` and
  /// `lane->elevation_bounds()`).
  static bool IsWithinLane(const maliput::api::GeoPositionT<T>& geo_position,
                           const maliput::api::Lane* lane);

  /// Returns `true` if and only if @p lane_position is within
  /// `linear_tolerance()` of the driveable bounds of @p lane and, in addition,
  /// `r` is within its lane bounds.
  static bool IsWithinLane(const maliput::api::LanePositionT<T>& lane_position,
                           const maliput::api::Lane* lane);

 private:
  // Returns the closest pose to the ego car given a `road`, the ego vehicle's
  // pose `ego_pose`, a PoseBundle of `traffic_poses`, the AheadOrBehind
  // specifier `side`, and a set of `branches` to be checked.  The return value
  // is the same as PoseSelector<T>::FindSingleClosestPose().
  static ClosestPose<T> FindSingleClosestInBranches(
      const maliput::api::RoadGeometry& road,
      const systems::rendering::PoseVector<T>& ego_pose,
      const systems::rendering::PoseBundle<T>& traffic_poses,
      const AheadOrBehind side, const std::vector<LaneEndDistance>& branches);

  // Returns the vector of branches along the sequence of default road segments
  // in a `road`, up to a given `scan_distance` in the ego vehicle's current
  // lane, given its PoseVector `ego_pose` and AheadOrBehind `side`. A vector of
  // LaneEndDistance is returned, whose elements are pairs where the first entry
  // is the distance along the s-coordinate from the ego vehicle to the branch
  // and second entry is the LaneEnd describing the branch.
  static std::vector<LaneEndDistance> FindConfluentBranches(
      const maliput::api::RoadGeometry& road,
      const systems::rendering::PoseVector<T>& ego_pose, const T& scan_distance,
      const AheadOrBehind side);

  // Returns true if `lane0` has an equal identifier as `lane1`, and false
  // otherwise.  The result is trivially false if either is nullptr.
  static bool IsEqual(const maliput::api::Lane* lane0,
                      const maliput::api::Lane* lane1);

  // Returns a LaneEndSet consisting of all LaneEnds attached to the provided
  // lane (specified in `lane_direction`) corresponding to all branches
  // connected to the end of the lane that is reached when traveling in the
  // `with_s` direction specified within `lane_direction`.  The return value
  // contains a null pointer if no default branch is found.
  static const maliput::api::LaneEndSet* GetIncomingLaneEnds(
      const LaneDirection& lane_direction);

  // Given a @p lane_direction, returns its default branch and updates @p
  // lane_direction to match the `lane` and `with_s` of that branch.  If there
  // is no default branch, the ongoing branch with index = 0 is selected.
  // Otherwise, returns `nullopt` and sets `lane_direction->lane` to `nullptr`.
  static optional<maliput::api::LaneEnd> GetDefaultOrFirstOngoingLane(
      LaneDirection* lane_direction);

  // Returns a RoadOdometry that contains an infinite `s` position, zero `r` and
  // `h` positions, and zero velocities. If @p lane_direction contains `with_s
  // == True`, a RoadOdometry containing an s-position at positive infinity is
  // returned; otherwise a negative-infinite position is returned.  For T =
  // AutoDiffXd, the derivatives of the returned RoadOdometry are made to be
  // coherent with respect to @p pose.
  static RoadOdometry<T> MakeInfiniteOdometry(
      const LaneDirection& lane_direction,
      const systems::rendering::PoseVector<T>& pose);

  // Returns positive infinity. For T = AutoDiffXd, the derivatives of the
  // the return value are made to be coherent with respect to @p pose.
  static T MakeInfiniteDistance(const systems::rendering::PoseVector<T>& pose);

  // Returns the distance (along the `s`-coordinate) from an end of a lane to a
  // @p lane_position in that lane, where the end is determined by the `with_s`
  // of the provided `lane_direction`.  Both `lane` and `with_s` are specified
  // in @p lane_direction.  Throws if any element of @p lane_position is not
  // within the respective bounds of `lane_direction.lane`.
  static T CalcLaneProgress(
      const LaneDirection& lane_direction,
      const maliput::api::LanePositionT<T>& lane_position);

  // Constructs a LaneDirection structure based on a vehicle's current @p lane,
  // @p lane_position, @p rotation (in global coordinates), and the @p side of
  // the car (ahead or behind) that traffic is being observed.  Note that
  // `LaneDirection::with_s` in the return argument is interpreted as the
  // direction along which targets are being observed (regardless of the ego
  // car's orientation): it is true if cars are being observed along the
  // `s`-direction and is false otherwise.
  static LaneDirection CalcLaneDirection(
      const maliput::api::Lane* lane,
      const maliput::api::LanePositionT<T>& lane_position,
      const Eigen::Quaternion<T>& rotation, AheadOrBehind side);

  // Helper that makes a GeoPosition from the provided Isometry3.
  static maliput::api::GeoPosition MakeGeoPosition(
      const Isometry3<T>& isometry);
};

}  // namespace automotive
}  // namespace drake
