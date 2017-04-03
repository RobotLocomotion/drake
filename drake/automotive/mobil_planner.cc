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

namespace {

static constexpr double kDefaultLargeAccel{1e6};  // m/s^2

}  // namespace

template <typename T>
MobilPlanner<T>::MobilPlanner(const RoadGeometry& road, bool initial_with_s)
    : IdmController<T>(road),
      with_s_(initial_with_s),
      lane_index_(this->DeclareAbstractOutputPort(
                          systems::Value<LaneDirection>(LaneDirection()))
                      .get_index()) {
  // Validate the provided RoadGeometry.
  DRAKE_DEMAND(this->road().num_junctions() > 0);
  DRAKE_DEMAND(this->road().junction(0)->num_segments() > 0);
  DRAKE_DEMAND(this->road().junction(0)->segment(0)->num_lanes() > 0);
  this->DeclareNumericParameter(MobilPlannerParameters<T>());
}

template <typename T>
const systems::OutputPortDescriptor<T>& MobilPlanner<T>::lane_output() const {
  return systems::System<T>::get_output_port(lane_index_);
}

template <typename T>
void MobilPlanner<T>::DoCalcOutput(const systems::Context<T>& context,
                                   systems::SystemOutput<T>* output) const {
  // Obtain the parameters.
  DRAKE_DEMAND(context.get_parameters().num_numeric_parameters() == 2);
  const IdmPlannerParameters<T>& idm_params =
      this->template GetNumericParameter<IdmPlannerParameters>(context, 0);
  const MobilPlannerParameters<T>& mobil_params =
      this->template GetNumericParameter<MobilPlannerParameters>(context, 1);

  // Obtain the input/output data structures.
  const PoseVector<T>* const ego_pose =
      this->template EvalVectorInput<PoseVector>(context,
                                                 this->ego_pose_index());
  DRAKE_ASSERT(ego_pose != nullptr);

  const FrameVelocity<T>* const ego_velocity =
      this->template EvalVectorInput<FrameVelocity>(context,
                                                    this->ego_velocity_index());
  DRAKE_ASSERT(ego_velocity != nullptr);

  const PoseBundle<T>* const traffic_poses =
      this->template EvalInputValue<PoseBundle<T>>(context,
                                                   this->traffic_index());
  DRAKE_ASSERT(traffic_poses != nullptr);

  systems::BasicVector<T>* const command_output_vector =
      output->GetMutableVectorData(this->driving_command_index());
  DRAKE_ASSERT(command_output_vector != nullptr);
  DrivingCommand<T>* const driving_command =
      dynamic_cast<DrivingCommand<T>*>(command_output_vector);
  DRAKE_ASSERT(driving_command != nullptr);

  this->ImplDoCalcOutput(*ego_pose, *ego_velocity, *traffic_poses, idm_params,
                         driving_command);

  LaneDirection* lane_direction =
      &output->GetMutableData(lane_index_)
           ->template GetMutableValue<LaneDirection>();
  DRAKE_ASSERT(lane_direction != nullptr);

  ImplDoCalcLane(*ego_pose, *ego_velocity, *traffic_poses, *driving_command,
                 idm_params, mobil_params, lane_direction);
}

template <typename T>
void MobilPlanner<T>::ImplDoCalcLane(
    const PoseVector<T>& ego_pose, const FrameVelocity<T>& ego_velocity,
    const PoseBundle<T>& traffic_poses,
    const DrivingCommand<T>& driving_command,
    const IdmPlannerParameters<T>& idm_params,
    const MobilPlannerParameters<T>& mobil_params,
    LaneDirection* lane_direction) const {
  DRAKE_DEMAND(idm_params.IsValid());
  DRAKE_DEMAND(mobil_params.IsValid());

  const RoadPosition& ego_position =
      pose_selector::CalcRoadPosition(this->road(), ego_pose.get_isometry());
  // Prepare a list of (possibly nullptr) Lanes to evaluate.
  std::pair<const Lane*, const Lane*> lanes = std::make_pair(
      ego_position.lane->to_left(), ego_position.lane->to_right());

  const Lane* lane = ego_position.lane;
  if (lanes.first != nullptr || lanes.second != nullptr) {
    const std::pair<T, T> incentives = ComputeIncentives(
        lanes, idm_params, mobil_params, ego_pose, ego_velocity, traffic_poses,
        driving_command.acceleration());
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

  const RoadPosition& ego_position =
      pose_selector::CalcRoadPosition(this->road(), ego_pose.get_isometry());
  DRAKE_DEMAND(ego_position.lane != nullptr);
  RoadOdometry<T> leading_odometry{};
  RoadOdometry<T> trailing_odometry{};
  std::tie(leading_odometry, trailing_odometry) =
      pose_selector::FindClosestPair(this->road(), ego_pose, traffic_poses);

  // Current acceleration of the ego car.
  const RoadOdometry<T>& ego_odometry =
      RoadOdometry<T>(ego_position, ego_velocity);
  // Current acceleration of the trailing car.
  const T trailing_this_old_accel =
      EvaluateIdm(idm_params, trailing_odometry, ego_odometry);
  // New acceleration of the trailing car if the ego were to change lanes.
  const T trailing_this_new_accel =
      EvaluateIdm(idm_params, trailing_odometry, leading_odometry);
  // Acceleration delta of the trailing car in the ego car's current lane.
  const T trailing_delta_accel_this =
      trailing_this_new_accel - trailing_this_old_accel;
  // Compute the incentive for the left lane.
  if (lanes.first != nullptr) {
    const OdometryPair& odometries = pose_selector::FindClosestPair(
        this->road(), ego_pose, traffic_poses, lanes.first);
    ComputeIncentiveOutOfLane(idm_params, mobil_params, odometries,
                              ego_odometry, ego_acceleration,
                              trailing_delta_accel_this, &incentives.first);
  }
  // Compute the incentive for the right lane.
  if (lanes.second != nullptr) {
    const OdometryPair& odometries = pose_selector::FindClosestPair(
        this->road(), ego_pose, traffic_poses, lanes.second);
    ComputeIncentiveOutOfLane(idm_params, mobil_params, odometries,
                              ego_odometry, ego_acceleration,
                              trailing_delta_accel_this, &incentives.second);
  }
  return incentives;
}

template <typename T>
void MobilPlanner<T>::ComputeIncentiveOutOfLane(
    const IdmPlannerParameters<T>& idm_params,
    const MobilPlannerParameters<T>& mobil_params,
    const OdometryPair& odometries, const RoadOdometry<T>& ego_odometry,
    const T& ego_old_accel, const T& trailing_delta_accel_this,
    T* incentive) const {
  RoadOdometry<T> leading_odometry{};
  RoadOdometry<T> trailing_odometry{};
  std::tie(leading_odometry, trailing_odometry) = odometries;
  // Acceleration of the ego car if it were to move to the neighboring lane.
  const T ego_new_accel =
      EvaluateIdm(idm_params, ego_odometry, leading_odometry);
  // Original acceleration of the trailing car in the neighboring lane.
  const T trailing_old_accel =
      EvaluateIdm(idm_params, trailing_odometry, leading_odometry);
  // Acceleration of the trailing car in the neighboring lane if the ego moves
  // here.
  const T trailing_new_accel =
      EvaluateIdm(idm_params, trailing_odometry, ego_odometry);
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
    const RoadOdometry<T>& ego_odometry,
    const RoadOdometry<T>& lead_car_odometry) const {
  const T& s_ego = ego_odometry.pos.s;
  const T& s_dot_ego = pose_selector::GetSVelocity(ego_odometry);
  const T& s_lead = lead_car_odometry.pos.s;
  const T& s_dot_lead = pose_selector::GetSVelocity(lead_car_odometry);

  const T delta = s_lead - s_ego;
  // Saturate the net_distance at distance_lower_bound away from the ego car to
  // prevent the IDM equation from producing near-singular solutions.
  // clang-format off
  // TODO(jadecastro): Move this to IdmPlanner::Evaluate().
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
