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
using automotive::pose_selector::RoadOdometry;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

namespace automotive {

static constexpr int kIdmParamsIndex{0};
static constexpr int kMobilParamsIndex{1};
static constexpr double kDefaultLargeAccel{1e6};  // m/s^2

template <typename T>
MobilPlanner<T>::MobilPlanner(const RoadGeometry& road,
                              const LaneDirection& initial_lane_direction)
    : road_(road), with_s_(initial_lane_direction.with_s) {
  // Validate the provided RoadGeometry.
  DRAKE_DEMAND(road_.num_junctions() > 0);
  DRAKE_DEMAND(road_.junction(0)->num_segments() > 0);
  DRAKE_DEMAND(road_.junction(0)->segment(0)->num_lanes() > 0);

  // Declare the input port for the ego car PoseVector.
  ego_pose_index_ =
      this->DeclareInputPort(systems::kVectorValued, PoseVector<T>::kSize)
          .get_index();
  // Declare the input port for the ego car FrameVelocity.
  ego_velocity_index_ =
      this->DeclareInputPort(systems::kVectorValued, FrameVelocity<T>::kSize)
          .get_index();
  // Declare the input port for the traffic car PoseBundle.
  traffic_index_ = this->DeclareAbstractInputPort().get_index();
  // Declare the DrivingCommand output port.
  command_index_ =
      this->DeclareVectorOutputPort(DrivingCommand<T>()).get_index();
  // Declare the Lane output port.  Infer Lane type from the provided
  // RoadGeometry.
  lane_index_ = this->DeclareAbstractOutputPort(
                        systems::Value<LaneDirection>(initial_lane_direction))
                    .get_index();

  this->DeclareNumericParameter(IdmPlannerParameters<T>());
  this->DeclareNumericParameter(MobilPlannerParameters<T>());
}

template <typename T>
MobilPlanner<T>::~MobilPlanner() {}

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
const systems::InputPortDescriptor<T>& MobilPlanner<T>::traffic_input() const {
  return systems::System<T>::get_input_port(traffic_index_);
}

template <typename T>
const systems::OutputPortDescriptor<T>&
MobilPlanner<T>::driving_command_output() const {
  return systems::System<T>::get_output_port(command_index_);
}

template <typename T>
const systems::OutputPortDescriptor<T>& MobilPlanner<T>::lane_output() const {
  return systems::System<T>::get_output_port(lane_index_);
}

template <typename T>
void MobilPlanner<T>::DoCalcOutput(const systems::Context<T>& context,
                                   systems::SystemOutput<T>* output) const {
  // Obtain the parameters.
  const IdmPlannerParameters<T>& idm_params =
      this->template GetNumericParameter<IdmPlannerParameters>(context,
                                                               kIdmParamsIndex);
  const MobilPlannerParameters<T>& mobil_params =
      this->template GetNumericParameter<MobilPlannerParameters>(
          context, kMobilParamsIndex);

  // Obtain the input/output data structures.
  const PoseVector<T>* const ego_pose =
      this->template EvalVectorInput<PoseVector>(
          context, this->ego_pose_input().get_index());
  DRAKE_ASSERT(ego_pose != nullptr);

  const FrameVelocity<T>* const ego_velocity =
      this->template EvalVectorInput<FrameVelocity>(context,
                                                    ego_velocity_index_);
  DRAKE_ASSERT(ego_velocity != nullptr);

  const PoseBundle<T>* const traffic_poses =
      this->template EvalInputValue<PoseBundle<T>>(
          context, this->traffic_input().get_index());
  DRAKE_ASSERT(traffic_poses != nullptr);

  LaneDirection* lane_direction =
      &output->GetMutableData(lane_index_)
           ->template GetMutableValue<LaneDirection>();

  ImplDoCalcLane(*ego_pose, *ego_velocity, *traffic_poses, idm_params,
                 mobil_params, lane_direction);

  systems::BasicVector<T>* const command_output_vector =
      output->GetMutableVectorData(driving_command_output().get_index());
  DRAKE_ASSERT(command_output_vector != nullptr);
  DrivingCommand<T>* const driving_command =
      dynamic_cast<DrivingCommand<T>*>(command_output_vector);
  DRAKE_ASSERT(driving_command != nullptr);

  ImplDoCalcAccel(*ego_pose, *ego_velocity, *traffic_poses, idm_params,
                  driving_command);
}

template <typename T>
void MobilPlanner<T>::ImplDoCalcLane(
    const PoseVector<T>& ego_pose, const FrameVelocity<T>& ego_velocity,
    const PoseBundle<T>& traffic_poses,
    const IdmPlannerParameters<T>& idm_params,
    const MobilPlannerParameters<T>& mobil_params,
    LaneDirection* lane_direction) const {
  DRAKE_DEMAND(idm_params.IsValid());
  DRAKE_DEMAND(mobil_params.IsValid());

  const RoadPosition& ego_position =
      pose_selector::CalcRoadPosition(road_, ego_pose.get_isometry());
  // Prepare a list of (possibly nullptr) Lanes to evaluate.
  std::pair<const Lane*, const Lane*> lanes = std::make_pair(
      ego_position.lane->to_left(), ego_position.lane->to_right());

  const Lane* lane = ego_position.lane;
  if (lanes.first != nullptr || lanes.second != nullptr) {
    const std::pair<T, T> incentives = ComputeIncentives(
        lanes, idm_params, mobil_params, ego_pose, ego_velocity, traffic_poses);
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
void MobilPlanner<T>::ImplDoCalcAccel(const PoseVector<T>& ego_pose,
                                      const FrameVelocity<T>& ego_velocity,
                                      const PoseBundle<T>& traffic_poses,
                                      const IdmPlannerParameters<T>& idm_params,
                                      DrivingCommand<T>* command) const {
  DRAKE_DEMAND(idm_params.IsValid());

  const RoadPosition& ego_position =
      pose_selector::CalcRoadPosition(road_, ego_pose.get_isometry());
  const RoadOdometry<T>& traffic_odometry =
      pose_selector::FindClosestLeading(road_, ego_pose, traffic_poses);

  // Output the acceleration command from the IDM equation and allocate the
  // result to either the throttle or brake.
  const T command_acceleration =
      EvaluateIdm(idm_params, {ego_position, ego_velocity}, traffic_odometry);
  command->set_acceleration(command_acceleration);
  command->set_steering_angle(0.);
}

template <typename T>
const std::pair<T, T> MobilPlanner<T>::ComputeIncentives(
    const std::pair<const Lane*, const Lane*> lanes,
    const IdmPlannerParameters<T>& idm_params,
    const MobilPlannerParameters<T>& mobil_params,
    const PoseVector<T>& ego_pose, const FrameVelocity<T>& ego_velocity,
    const PoseBundle<T>& traffic_poses) const {
  // Initially disincentivize all neighboring lane options.
  std::pair<T, T> incentives(-kDefaultLargeAccel, -kDefaultLargeAccel);

  const RoadPosition& ego_position =
      pose_selector::CalcRoadPosition(road_, ego_pose.get_isometry());
  DRAKE_DEMAND(ego_position.lane != nullptr);
  RoadOdometry<T> leading_odometry{};
  RoadOdometry<T> trailing_odometry{};
  std::tie(leading_odometry, trailing_odometry) =
      pose_selector::FindClosestPair(road_, ego_pose, traffic_poses);

  // Current acceleration of the ego car.
  const RoadOdometry<T>& ego_odometry =
      RoadOdometry<T>(ego_position, ego_velocity);
  const T ego_old_accel =
      EvaluateIdm(idm_params, ego_odometry, leading_odometry);
  // Current acceleration of the trailing car.
  const T trailing_this_old_accel =
      EvaluateIdm(idm_params, trailing_odometry, ego_odometry);
  // New acceleration of the trailing car if the ego were to change lanes.
  const T trailing_this_new_accel =
      EvaluateIdm(idm_params, trailing_odometry, leading_odometry);
  const T trailing_delta_accel_this =
      trailing_this_new_accel - trailing_this_old_accel;
  // Compute the incentive for the left lane.
  if (lanes.first != nullptr) {
    const OdometryPair& leading_trailing = pose_selector::FindClosestPair(
        road_, ego_pose, traffic_poses, lanes.first);
    CompareOutOfLane(idm_params, mobil_params, leading_trailing, ego_odometry,
                     ego_old_accel, trailing_delta_accel_this,
                     &incentives.first);
  }
  // Compute the incentive for the right lane.
  if (lanes.second != nullptr) {
    const OdometryPair& leading_trailing = pose_selector::FindClosestPair(
        road_, ego_pose, traffic_poses, lanes.second);
    CompareOutOfLane(idm_params, mobil_params, leading_trailing, ego_odometry,
                     ego_old_accel, trailing_delta_accel_this,
                     &incentives.second);
  }

  return incentives;
}

template <typename T>
void MobilPlanner<T>::CompareOutOfLane(
    const IdmPlannerParameters<T>& idm_params,
    const MobilPlannerParameters<T>& mobil_params,
    const OdometryPair& leading_trailing, const RoadOdometry<T>& ego_odometry,
    const double& ego_old_accel, const double& trailing_delta_accel_this,
    double* incentive) const {
  RoadOdometry<T> leading_odometry{};
  RoadOdometry<T> trailing_odometry{};
  std::tie(leading_odometry, trailing_odometry) = leading_trailing;
  // Acceleration of the ego car if it were to move to this lane.
  const T ego_new_accel =
      EvaluateIdm(idm_params, ego_odometry, leading_odometry);
  // Original acceleration of the trailing car in this lane.
  const T trailing_old_accel =
      EvaluateIdm(idm_params, trailing_odometry, leading_odometry);
  // Acceleration of the trailing car in this lane if the ego moves here.
  const T trailing_new_accel =
      EvaluateIdm(idm_params, trailing_odometry, ego_odometry);
  const T trailing_delta_accel_other = trailing_new_accel - trailing_old_accel;
  const T ego_delta_accel = ego_new_accel - ego_old_accel;

  // Do not switch to this lane if it discomforts the trailing car too much.
  if (trailing_old_accel < -mobil_params.max_deceleration()) return;

  // Compute the incentive as a weighted sum of the net accelerations for
  // the ego and each immediate neighbor.
  *incentive = ego_delta_accel +
               mobil_params.p() *
                   (trailing_delta_accel_other + trailing_delta_accel_this);
}

template <typename T>
const T MobilPlanner<T>::EvaluateIdm(
    const IdmPlannerParameters<T>& idm_params,
    const RoadOdometry<T>& ego_odometry,
    const RoadOdometry<T>& lead_car_odometry) const {
  const T& s_ego = ego_odometry.pos.s;
  const T& s_dot_ego = pose_selector::GetSVelocity(ego_odometry);
  const T& s_lead = lead_car_odometry.pos.s;
  const T& s_dot_lead = pose_selector::GetSVelocity(lead_car_odometry);

  const T delta = s_lead - s_ego;
  // Saturate the net_distance at distance_lower_bound away from the ego car.
  // clang-format off
  const T net_distance =
      cond(delta > T(0.), saturate(delta - idm_params.bloat_diameter(),
                                   idm_params.distance_lower_limit(),
                                   std::numeric_limits<T>::infinity()),
                          saturate(delta + idm_params.bloat_diameter(),
                                   -std::numeric_limits<T>::infinity(),
                                   -idm_params.distance_lower_limit()));
  // clang-format on
  DRAKE_DEMAND(std::abs(net_distance) >= idm_params.distance_lower_limit());
  const T closing_velocity = s_dot_ego - s_dot_lead;

  return IdmPlanner<T>::Evaluate(idm_params, s_dot_ego, net_distance,
                                 closing_velocity);
}

// These instantiations must match the API documentation in mobil_planner.h.
template class MobilPlanner<double>;

}  // namespace automotive
}  // namespace drake
