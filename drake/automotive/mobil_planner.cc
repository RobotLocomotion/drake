#include "drake/automotive/mobil_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_formula.h"
#include "drake/math/saturate.h"

namespace drake {

using maliput::api::GeoPosition;
using maliput::api::Lane;
using maliput::api::LanePosition;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using math::saturate;
using systems::BasicVector;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

namespace automotive {

namespace {

static constexpr int kIdmParamsIndex{0};
static constexpr int kMobilParamsIndex{1};
static constexpr double kDefaultLargeAccel{1e6};  // m/s^2

}  // namespace

template <typename T>
MobilPlanner<T>::MobilPlanner(const RoadGeometry& road, bool initial_with_s)
    : road_(road),
      with_s_(initial_with_s),
      ego_pose_index_{
          this->DeclareVectorInputPort(PoseVector<T>()).get_index()},
      ego_velocity_index_{
          this->DeclareVectorInputPort(FrameVelocity<T>()).get_index()},
      ego_acceleration_index_{
          this->DeclareVectorInputPort(BasicVector<T>(1)).get_index()},
      traffic_index_{this->DeclareAbstractInputPort().get_index()},
      lane_index_{
          this->DeclareAbstractOutputPort(&MobilPlanner::CalcLaneDirection)
              .get_index()} {
  // Validate the provided RoadGeometry.
  DRAKE_DEMAND(road_.num_junctions() > 0);
  DRAKE_DEMAND(road_.junction(0)->num_segments() > 0);
  DRAKE_DEMAND(road_.junction(0)->segment(0)->num_lanes() > 0);
  this->DeclareNumericParameter(IdmPlannerParameters<T>());
  this->DeclareNumericParameter(MobilPlannerParameters<T>());
}

template <typename T>
const systems::InputPortDescriptor<T>& MobilPlanner<T>::ego_pose_input() const {
  return systems::System<T>::get_input_port(ego_pose_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>& MobilPlanner<T>::ego_velocity_input()
    const {
  return systems::System<T>::get_input_port(ego_velocity_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>& MobilPlanner<T>::ego_acceleration_input()
    const {
  return systems::System<T>::get_input_port(ego_acceleration_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>& MobilPlanner<T>::traffic_input() const {
  return systems::System<T>::get_input_port(traffic_index_);
}

template <typename T>
const systems::OutputPort<T>& MobilPlanner<T>::lane_output() const {
  return systems::System<T>::get_output_port(lane_index_);
}

template <typename T>
void MobilPlanner<T>::CalcLaneDirection(const systems::Context<T>& context,
                                        LaneDirection* lane_direction) const {
  // Obtain the parameters.
  const IdmPlannerParameters<T>& idm_params =
      this->template GetNumericParameter<IdmPlannerParameters>(context,
                                                               kIdmParamsIndex);
  const MobilPlannerParameters<T>& mobil_params =
      this->template GetNumericParameter<MobilPlannerParameters>(
          context, kMobilParamsIndex);

  // Obtain the input/output data structures.
  const PoseVector<T>* const ego_pose =
      this->template EvalVectorInput<PoseVector>(context, ego_pose_index_);
  DRAKE_ASSERT(ego_pose != nullptr);

  const FrameVelocity<T>* const ego_velocity =
      this->template EvalVectorInput<FrameVelocity>(context,
                                                    ego_velocity_index_);
  DRAKE_ASSERT(ego_velocity != nullptr);

  const BasicVector<T>* const ego_accel_command =
      this->template EvalVectorInput<BasicVector>(context,
                                                  ego_acceleration_index_);
  DRAKE_ASSERT(ego_accel_command != nullptr);

  const PoseBundle<T>* const traffic_poses =
      this->template EvalInputValue<PoseBundle<T>>(context, traffic_index_);
  DRAKE_ASSERT(traffic_poses != nullptr);

  ImplCalcLaneDirection(*ego_pose, *ego_velocity, *traffic_poses,
                        *ego_accel_command, idm_params, mobil_params,
                        lane_direction);
}

template <typename T>
void MobilPlanner<T>::ImplCalcLaneDirection(
    const PoseVector<T>& ego_pose, const FrameVelocity<T>& ego_velocity,
    const PoseBundle<T>& traffic_poses, const BasicVector<T>& ego_accel_command,
    const IdmPlannerParameters<T>& idm_params,
    const MobilPlannerParameters<T>& mobil_params,
    LaneDirection* lane_direction) const {
  DRAKE_DEMAND(idm_params.IsValid());
  DRAKE_DEMAND(mobil_params.IsValid());

  const RoadPosition ego_position =
      road_.ToRoadPosition(
          GeoPosition::FromXyz(ego_pose.get_isometry().translation()),
          nullptr, nullptr, nullptr);
  // Prepare a list of (possibly nullptr) Lanes to evaluate.
  std::pair<const Lane*, const Lane*> lanes = std::make_pair(
      ego_position.lane->to_left(), ego_position.lane->to_right());

  const Lane* lane = ego_position.lane;
  if (lanes.first != nullptr || lanes.second != nullptr) {
    const std::pair<T, T> incentives =
        ComputeIncentives(lanes, idm_params, mobil_params, ego_pose,
                          ego_velocity, traffic_poses, ego_accel_command[0]);
    // Switch to the lane with the highest incentive score greater than zero,
    // staying in the same lane if under the threshold.
    const T threshold = mobil_params.threshold();
    if (incentives.first >= incentives.second)
      lane = (incentives.first > threshold) ? lanes.first : ego_position.lane;
    else
      lane = (incentives.second > threshold) ? lanes.second : ego_position.lane;
  }
  *lane_direction = LaneDirection(lane, with_s_);
  // N.B. Assumes neighboring lanes are all confluent (i.e. with_s points in the
  // same direction).
}

template <typename T>
const std::pair<T, T> MobilPlanner<T>::ComputeIncentives(
    const std::pair<const Lane*, const Lane*> lanes,
    const IdmPlannerParameters<T>& idm_params,
    const MobilPlannerParameters<T>& mobil_params,
    const PoseVector<T>& ego_pose, const FrameVelocity<T>& ego_velocity,
    const PoseBundle<T>& traffic_poses, const T& ego_acceleration) const {
  // Initially disincentivize both neighboring lane options.  N.B. The first and
  // second elements correspond to the left and right lanes, respectively.
  std::pair<T, T> incentives(-kDefaultLargeAccel, -kDefaultLargeAccel);

  const RoadPosition ego_position =
      road_.ToRoadPosition(
          GeoPosition::FromXyz(ego_pose.get_isometry().translation()),
          nullptr, nullptr, nullptr);
  DRAKE_DEMAND(ego_position.lane != nullptr);
  const ClosestPoses current_closest_poses = PoseSelector<T>::FindClosestPair(
      ego_position.lane, ego_pose, traffic_poses,
      idm_params.scan_ahead_distance());
  // Construct ClosestPose containers for the leading, trailing, and ego car.
  const ClosestPose<T>& leading_closest_pose =
      current_closest_poses.at(AheadOrBehind::kAhead);
  const ClosestPose<T>& trailing_closest_pose =
      current_closest_poses.at(AheadOrBehind::kBehind);
  const ClosestPose<T> ego_closest_pose(
      RoadOdometry<T>(ego_position, ego_velocity), 0.);

  // Current acceleration of the trailing car.
  const T trailing_this_old_accel =
      EvaluateIdm(idm_params, trailing_closest_pose, ego_closest_pose);
  // New acceleration of the trailing car if the ego were to change lanes.
  const T trailing_this_new_accel =
      EvaluateIdm(idm_params, trailing_closest_pose, leading_closest_pose);
  // Acceleration delta of the trailing car in the ego car's current lane.
  const T trailing_delta_accel_this =
      trailing_this_new_accel - trailing_this_old_accel;
  // Compute the incentive for the left lane.
  if (lanes.first != nullptr) {
    const ClosestPoses left_closest_poses = PoseSelector<T>::FindClosestPair(
        lanes.first, ego_pose, traffic_poses, idm_params.scan_ahead_distance());
    ComputeIncentiveOutOfLane(idm_params, mobil_params, left_closest_poses,
                              ego_closest_pose, ego_acceleration,
                              trailing_delta_accel_this, &incentives.first);
  }
  // Compute the incentive for the right lane.
  if (lanes.second != nullptr) {
    const ClosestPoses right_closest_poses =
        PoseSelector<T>::FindClosestPair(lanes.second, ego_pose, traffic_poses,
                                         idm_params.scan_ahead_distance());
    ComputeIncentiveOutOfLane(idm_params, mobil_params, right_closest_poses,
                              ego_closest_pose, ego_acceleration,
                              trailing_delta_accel_this, &incentives.second);
  }
  return incentives;
}

template <typename T>
void MobilPlanner<T>::ComputeIncentiveOutOfLane(
    const IdmPlannerParameters<T>& idm_params,
    const MobilPlannerParameters<T>& mobil_params,
    const ClosestPoses& closest_poses, const ClosestPose<T>& ego_closest_pose,
    const T& ego_old_accel, const T& trailing_delta_accel_this,
    T* incentive) const {
  const ClosestPose<T>& leading_closest_pose =
      closest_poses.at(AheadOrBehind::kAhead);
  const ClosestPose<T>& trailing_closest_pose =
      closest_poses.at(AheadOrBehind::kBehind);

  // Acceleration of the ego car if it were to move to the neighboring lane.
  const T ego_new_accel =
      EvaluateIdm(idm_params, ego_closest_pose, leading_closest_pose);
  // Original acceleration of the trailing car in the neighboring lane.
  const T trailing_old_accel =
      EvaluateIdm(idm_params, trailing_closest_pose, leading_closest_pose);
  // Acceleration of the trailing car in the neighboring lane if the ego moves
  // here.
  const T trailing_new_accel =
      EvaluateIdm(idm_params, trailing_closest_pose, ego_closest_pose);
  // Acceleration delta of the trailing car in the neighboring (other) lane.
  const T trailing_delta_accel_other = trailing_new_accel - trailing_old_accel;
  const T ego_delta_accel = ego_new_accel - ego_old_accel;

  // Do not switch to this lane if it discomforts the trailing car too much.
  if (trailing_new_accel < -mobil_params.max_deceleration()) return;

  // Compute the incentive as a weighted sum of the net accelerations for
  // the ego and each immediate neighbor.
  *incentive = ego_delta_accel +
               mobil_params.p() *
                   (trailing_delta_accel_other + trailing_delta_accel_this);
}

template <typename T>
const T MobilPlanner<T>::EvaluateIdm(
    const IdmPlannerParameters<T>& idm_params,
    const ClosestPose<T>& trailing_closest_pose,
    const ClosestPose<T>& leading_closest_pose) const {
  using std::abs;

  const T headway_distance =
      leading_closest_pose.distance + trailing_closest_pose.distance;
  // Saturate the net_distance at distance_lower_limit away from the ego car to
  // prevent the IDM equation from producing near-singular solutions.
  // TODO(jadecastro): Move this to IdmPlanner::Evaluate().
  const T net_distance =
      cond(headway_distance >= T(0.),
           saturate(headway_distance - idm_params.bloat_diameter(),
                    idm_params.distance_lower_limit(),
                    std::numeric_limits<T>::infinity()),
           saturate(headway_distance + idm_params.bloat_diameter(),
                    -std::numeric_limits<T>::infinity(),
                    -idm_params.distance_lower_limit()));
  DRAKE_DEMAND(std::abs(net_distance) >= idm_params.distance_lower_limit());

  const RoadOdometry<T> trailing_car_odometry(
      {trailing_closest_pose.odometry.lane, trailing_closest_pose.odometry.pos},
      trailing_closest_pose.odometry.vel);
  const T& s_dot_behind =
      (abs(trailing_car_odometry.pos.s()) == std::numeric_limits<T>::infinity())
          ? 0.
          : PoseSelector<T>::GetSigmaVelocity(trailing_car_odometry);
  const RoadOdometry<T> leading_car_odometry(
      {leading_closest_pose.odometry.lane, leading_closest_pose.odometry.pos},
      leading_closest_pose.odometry.vel);
  const T& s_dot_ahead =
      (abs(leading_car_odometry.pos.s()) == std::numeric_limits<T>::infinity())
          ? 0.
          : PoseSelector<T>::GetSigmaVelocity(leading_car_odometry);
  const T closing_velocity = s_dot_behind - s_dot_ahead;

  return IdmPlanner<T>::Evaluate(idm_params, s_dot_behind, net_distance,
                                 closing_velocity);
}

// These instantiations must match the API documentation in mobil_planner.h.
template class MobilPlanner<double>;

}  // namespace automotive
}  // namespace drake
