#pragma once

#include <memory>
#include <utility>

#include <Eigen/Geometry>

#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// Contains the position of the vehicle with respect to a lane in a road, along
/// with its velocity vector in the world frame.
template <typename T>
struct RoadOdometry {
  /// Default constructor.
  RoadOdometry() = default;
  /// Fully-parameterized constructor.
  RoadOdometry(const maliput::api::RoadPosition& road_position,
               const systems::rendering::FrameVelocity<T>& frame_velocity)
      : lane(road_position.lane), pos(road_position.pos), vel(frame_velocity) {}

  const maliput::api::Lane* lane{};
  maliput::api::LanePosition pos{};
  systems::rendering::FrameVelocity<T> vel{};
};

/// Specifies whether to assess the cars ahead or behind the ego car at its
/// current orientation with respect to its lane.
enum class WhichSide { kAhead = 0, kBehind = 1 };

/// PoseSelector is a class that provides the relevant pose or poses with
/// respect to a given ego vehicle driving within a given maliput road geometry.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - drake::AutoDiffXd
///
/// They are already available to link against in the containing library.
///
/// @ingroup automotive_controllers
template <typename T>
class PoseSelector {
 public:
  typedef typename std::pair<const T, const maliput::api::LaneEnd>
  LaneEndDistance;

  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PoseSelector)

  PoseSelector() = delete;

  /// Returns the leading and trailing vehicles that have closest
  /// `s`-coordinates in a given @p traffic_lane to an ego vehicle as if the ego
  /// vehicle, at its current `s`-position, is considered to be in @p lane.
  /// This function is used, for instance, as logic for lane-change planners
  /// (e.g. MOBIL).  The ego car's pose @p ego_pose, its geo-space velocity @p
  /// ego_velocity and the poses of the traffic cars (@p traffic_poses) are
  /// provided.  The parameter @p scan_distance determines the distance along
  /// the sequence of lanes to scan before declaring the car is at infinite
  /// distance ahead (resp. behind).  If no leading/trailing vehicles are seen
  /// within @p traffic_lane, `s`-positions are taken to be at infinite
  /// distances away from the ego car.  If a non-null @p distances is given,
  /// then it is populated with a pair of distances (ahead and behind) that are
  /// closest.  Infinite distances are returned if no traffic cars are found.
  ///
  /// @return A pair of leading/trailing RoadOdometries. Note that when no
  /// vehicle is detected in front of (resp. behind) the ego vehicle, the
  /// respective RoadPosition will contain an `s`-value of positive
  /// (resp. negative) infinity (`std::numeric_limits<double>::infinity()`).
  /// Any traffic poses that are redunant with `ego_pose` (i.e. have the same
  /// RoadPosition as the ego car) are discarded.
  ///
  /// The RoadGeometry from which @p lane is drawn is required to have default
  /// branches set for all branches in the road network.
  ///
  /// Note: This function is lossy with respect to AutoDiffXd partial
  /// derivatives.
  static const std::pair<const RoadOdometry<T>, const RoadOdometry<T>>
  FindClosestPair(const maliput::api::Lane* const lane,
                  const systems::rendering::PoseVector<T>& ego_pose,
                  const systems::rendering::FrameVelocity<T>& ego_velocity,
                  const systems::rendering::PoseBundle<T>& traffic_poses,
                  const T& scan_distance, std::pair<T, T>* distances = nullptr);

  /// Same as PoseSelector::FindClosestPair() except that: (1) it only considers
  /// the ego car's lane and (2) it returns a single the RoadOdometry of either
  /// the vehicle ahead (WhichSide::kAhead) or behind (WhichSide::kBehind).
  ///
  /// Note that when no car is detected in front of the ego car, the returned
  /// RoadOdometry will contain an `s`-value of
  /// `std::numeric_limits<double>::infinity()`.
  ///
  /// Note: This function is lossy with respect to AutoDiffXd partial
  /// derivatives.
  static const RoadOdometry<T> FindSingleClosestPose(
          const maliput::api::Lane* const lane,
          const systems::rendering::PoseVector<T>& ego_pose,
          const systems::rendering::FrameVelocity<T>& ego_velocity,
          const systems::rendering::PoseBundle<T>& traffic_poses,
          const T& scan_distance, const WhichSide side, T* distance = nullptr);

  /// Extracts the vehicle's `s`-direction velocity based on its RoadOdometry @p
  /// road_odom in the Lane coordinate frame.  Assumes the road has zero
  /// elevation and superelevation.
  ///
  /// Note: This function is lossy with respect to AutoDiffXd partial
  /// derivatives.
  static const T GetSigmaVelocity(const RoadOdometry<T>& road_odometry);

  // Returns `true` if within the given lane, and `false` otherwise.  @p
  // geo_position is the geo-space coordinates of the vehicle, and @p lane is
  // the lane within which `geo_position` will be checked for membership.
  static bool IsWithinLane(const maliput::api::GeoPosition& geo_position,
                           const maliput::api::Lane* lane);

 private:
  // Mutates `lane_direction` according to the next default lane based on the
  // current lane and travel direction contained within `lane_direction`.
  // `lane_direction` contains a null pointer in the `lane` field if no default
  // branch is found.
  static std::unique_ptr<maliput::api::LaneEnd> get_default_ongoing_lane(
      LaneDirection* lane_direction);

  // Assign the with default positions extending to, respectively, positive and
  // negative infinity and with zero velocities.
  static const RoadOdometry<T> set_default_odometry(
      const maliput::api::Lane* const Lane, const WhichSide side);
};

}  // namespace automotive
}  // namespace drake
