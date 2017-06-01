#pragma once

#include <map>
#include <memory>
#include <utility>

#include <Eigen/Geometry>

#include "drake/automotive/gen/idm_planner_parameters.h"
#include "drake/automotive/gen/mobil_planner_parameters.h"
#include "drake/automotive/idm_planner.h"
#include "drake/automotive/lane_direction.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/pose_selector.h"
#include "drake/automotive/road_odometry.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/rendering/pose_bundle.h"
#include "drake/systems/rendering/pose_vector.h"

namespace drake {
namespace automotive {

/// MOBIL (Minimizing Overall Braking Induced by Lane Changes) [1] is a planner
/// that minimizes braking requirement for the ego car while also minimizing
/// (per a weighting factor) the braking requirements of any trailing cars
/// within the ego car's immediate neighborhood.  The weighting factor
/// encapsulates the politeness of the ego car to the surrounding traffic.
/// Neighboring cars are defined as those cars immediately ahead and behind the
/// ego, in the current lane and any adjacent lanes; these are determined from
/// the PoseSelector logic applied to a multi-lane Maliput road.
///
/// The induced braking by the ego car and the car following immediately behind
/// it is compared with the induced braking by the ego and its new follower if
/// the ego were to move to any of the neighboring lanes.  The choice that
/// minimizes the induced braking - alternatively maximizes the ego car's
/// "incentive" (the weighted sum of accelerations that the ego car and its
/// neighbors gain by changing lanes) - is chosen as the new lane request.  The
/// request is expressed as a LaneDirection, that references a valid lane in the
/// provided RoadGeometry and the direction of travel.
///
/// Assumptions:
///   1) The planner supports only symmetric lane change rules, without giving
///      preference to lanes to the left or right.
///   2) The planner assumes all traffic behaves according to the Intelligent
///      Driver Model (IDM).
///   3) All neighboring lanes are confluent (i.e. with_s points in the same
///      direction).
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
///
/// They are already available to link against in the containing library.
///
/// Input Port 0: A PoseVector for the ego car.
///   (InputPortDescriptor getter: ego_pose_input())
///
/// Input Port 1: A FrameVelocity for the ego car.
///   (InputPortDescriptor getter: ego_velocity_input())
///
/// Input Port 2: A BasicVector containing the ego car's commanded acceleration
///   value intercepted from the vehicle's controller (e.g. IdmController).
///   (InputPortDescriptor getter: ego_acceleration_input())
///
/// Input Port 3: A PoseBundle for the traffic cars, possibly including the ego
///   car's pose.
///   (InputPortDescriptor getter: traffic_input())
///
/// Output Port 0: A LaneDirection containing a lane that the ego vehicle must
///   move into and the direction of travel with respect to the lane's canonical
///   direction of travel.  LaneDirection must be consistent with the provided
///   road.
///   (OutputPortDescriptor getter: lane_output())
///
/// @ingroup automotive_controllers
///
/// [1] Arne Kesting, Martin Treiber and Dirk Helbing, MOBIL: General
///     Lane-Changing Model for Car-Following Models, Journal of the
///     Transportation Research Board, v1999, 2007, pp 86-94.
///     http://trrjournalonline.trb.org/doi/abs/10.3141/1999-10.
template <typename T>
class MobilPlanner : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MobilPlanner)

  /// A constructor that initializes the MOBIL planner.
  /// @param road is the pre-defined RoadGeometry.
  /// @param initial_with_s is the initial direction of travel in the lane
  /// corresponding to the ego vehicle's initial state.
  explicit MobilPlanner(const maliput::api::RoadGeometry& road,
                        bool initial_with_s);

  /// See the class description for details on the following input ports.
  /// @{
  const systems::InputPortDescriptor<T>& ego_pose_input() const;
  const systems::InputPortDescriptor<T>& ego_velocity_input() const;
  const systems::InputPortDescriptor<T>& ego_acceleration_input() const;
  const systems::InputPortDescriptor<T>& traffic_input() const;
  const systems::OutputPortDescriptor<T>& lane_output() const;
  /// @}

 private:
  void DoCalcOutput(const systems::Context<T>& context,
                    systems::SystemOutput<T>* output) const override;

  // Performs the calculations for the lane_output() port.
  void ImplDoCalcLane(const systems::rendering::PoseVector<T>& ego_pose,
                      const systems::rendering::FrameVelocity<T>& ego_velocity,
                      const systems::rendering::PoseBundle<T>& traffic_poses,
                      const systems::BasicVector<T>& ego_accel_command,
                      const IdmPlannerParameters<T>& idm_params,
                      const MobilPlannerParameters<T>& mobil_params,
                      LaneDirection* lane_direction) const;

  // Computes a pair of incentive measures for the provided neighboring lanes.
  // The first and second elements in `lanes` correspond to, respectively, a
  // pair of lanes included in the incentive query.  The respective incentives
  // for these lanes are returned as the first and second elements in the return
  // value.
  const std::pair<T, T> ComputeIncentives(
      const std::pair<const maliput::api::Lane*, const maliput::api::Lane*>
          lanes,
      const IdmPlannerParameters<T>& idm_params,
      const MobilPlannerParameters<T>& mobil_params,
      const systems::rendering::PoseVector<T>& ego_pose,
      const systems::rendering::FrameVelocity<T>& ego_velocity,
      const systems::rendering::PoseBundle<T>& traffic_poses,
      const T& ego_acceleration) const;

  // Computes a pair of incentive measures that consider the leading and
  // trailing vehicles that are closest to the pre-computed result in the
  // current lane.  `closest_poses` contains the odometries and relative
  // distances to the leading and trailing cars.
  void ComputeIncentiveOutOfLane(
      const IdmPlannerParameters<T>& idm_params,
      const MobilPlannerParameters<T>& mobil_params,
      const std::map<AheadOrBehind, const ClosestPose<T>>& closest_poses,
      const RoadOdometry<T>& ego_odometry, const T& ego_old_accel,
      const T& trailing_delta_accel_this, T* incentive) const;

  // Computes an acceleration based on the IDM equation (via a call to
  // IdmPlanner::Eval()).
  const T EvaluateIdm(const IdmPlannerParameters<T>& idm_params,
                      const RoadOdometry<T>& ego_odometry,
                      const RoadOdometry<T>& lead_car_odometry,
                      const T& headway_distance) const;

  const maliput::api::RoadGeometry& road_;
  const bool with_s_{true};

  // Indices for the input / output ports.
  const int ego_pose_index_{};
  const int ego_velocity_index_{};
  const int ego_acceleration_index_{};
  const int traffic_index_{};
  const int lane_index_{};
};

}  // namespace automotive
}  // namespace drake
