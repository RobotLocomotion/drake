#include "drake/automotive/idm_controller.h"

#include <limits>
#include <utility>
#include <vector>

#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_formula.h"
#include "drake/math/saturate.h"

namespace drake {

using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using maliput::api::Rotation;
using math::saturate;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

namespace automotive {

static constexpr int kIdmParamsIndex{0};
static constexpr int kCarParamsIndex{1};

template <typename T>
IdmController<T>::IdmController(const RoadGeometry& road) : road_(road) {
  // Declare the ego car pose input.
  ego_pose_index_ = this->DeclareVectorInputPort(PoseVector<T>()).get_index();
  // Declare the ego car velocity input.
  ego_velocity_index_ =
      this->DeclareVectorInputPort(FrameVelocity<T>()).get_index();
  // Declare the traffic car pose bundle input.
  traffic_index_ = this->DeclareAbstractInputPort().get_index();
  // Declare the output port.
  this->DeclareVectorOutputPort(DrivingCommand<T>());
}

template <typename T>
IdmController<T>::~IdmController() {}

template <typename T>
const systems::InputPortDescriptor<T>& IdmController<T>::ego_pose_input()
    const {
  return systems::System<T>::get_input_port(ego_pose_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>& IdmController<T>::ego_velocity_input()
    const {
  return systems::System<T>::get_input_port(ego_velocity_index_);
}

template <typename T>
const systems::InputPortDescriptor<T>& IdmController<T>::traffic_input() const {
  return systems::System<T>::get_input_port(traffic_index_);
}

template <typename T>
void IdmController<T>::DoCalcOutput(const systems::Context<T>& context,
                                    systems::SystemOutput<T>* output) const {
  // Obtain the parameters.
  const IdmPlannerParameters<T>& idm_params =
      this->template GetNumericParameter<IdmPlannerParameters>(context,
                                                               kIdmParamsIndex);
  const SimpleCarConfig<T>& car_params =
      this->template GetNumericParameter<SimpleCarConfig>(context,
                                                          kCarParamsIndex);

  // Obtain the input/output data structures.
  const PoseVector<T>* const ego_pose =
      this->template EvalVectorInput<PoseVector>(context, ego_pose_index_);
  DRAKE_ASSERT(ego_pose != nullptr);

  const FrameVelocity<T>* const ego_velocity =
      this->template EvalVectorInput<FrameVelocity>(context,
                                                    ego_velocity_index_);
  DRAKE_ASSERT(ego_velocity != nullptr);

  const PoseBundle<T>* const traffic_poses =
      this->template EvalInputValue<PoseBundle<T>>(context, traffic_index_);
  DRAKE_ASSERT(traffic_poses != nullptr);

  systems::BasicVector<T>* const command_output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(command_output_vector != nullptr);
  DrivingCommand<T>* const driving_command =
      dynamic_cast<DrivingCommand<T>*>(command_output_vector);
  DRAKE_ASSERT(driving_command != nullptr);

  ImplDoCalcOutput(*ego_pose, *ego_velocity, *traffic_poses, idm_params,
                   car_params, driving_command);
}

template <typename T>
void IdmController<T>::ImplDoCalcOutput(
    const PoseVector<T>& ego_pose, const FrameVelocity<T>& ego_velocity,
    const PoseBundle<T>& traffic_poses,
    const IdmPlannerParameters<T>& idm_params,
    const SimpleCarConfig<T>& car_params, DrivingCommand<T>* command) const {
  // Find the single closest car ahead.
  const RoadOdometry<T>& lead_car_odom =
      pose_selector::FindClosestLeading(road_, ego_pose, traffic_poses);
  const RoadPosition ego_position =
      pose_selector::CalcRoadPosition(road_, ego_pose.get_isometry());

  const T& s_ego = ego_position.pos.s;
  const T& s_dot_ego =
      GetSVelocity(RoadOdometry<double>(ego_position, ego_velocity));
  const T& s_lead = lead_car_odom.pos.s;
  const T& s_dot_lead = GetSVelocity(lead_car_odom);

  // Saturate the net_distance at distance_lower_bound away from the ego car to
  // avoid near-singular solutions inherent to the IDM equation.
  const T net_distance = saturate(s_lead - s_ego - idm_params.bloat_diameter(),
                                  idm_params.distance_lower_limit(),
                                  std::numeric_limits<T>::infinity());
  const T closing_velocity = s_dot_ego - s_dot_lead;

  // Compute the acceleration command from the IDM equation and allocate the
  // normalized result to either the throttle or brake.
  const T command_acceleration = IdmPlanner<T>::Evaluate(
      idm_params, s_dot_ego, net_distance, closing_velocity);
  const T normalized_accel = command_acceleration / car_params.max_velocity();
  command->set_throttle(
      cond(normalized_accel < T(0.), T(0.), normalized_accel));
  command->set_brake(cond(normalized_accel >= T(0.), T(0.), -normalized_accel));
  command->set_steering_angle(0.);
}

template <typename T>
double IdmController<T>::GetSVelocity(const RoadOdometry<T>& road_odom) const {
  Rotation rot = road_odom.lane->GetOrientation(road_odom.pos);
  const T vx = road_odom.vel.get_velocity().translational().x();
  const T vy = road_odom.vel.get_velocity().translational().y();

  return vx * std::cos(rot.yaw) + vy * std::sin(rot.yaw);
}

template <typename T>
std::unique_ptr<systems::Parameters<T>> IdmController<T>::AllocateParameters()
    const {
  std::vector<std::unique_ptr<systems::BasicVector<T>>> params;
  params.insert(params.begin() + kIdmParamsIndex,
                std::make_unique<IdmPlannerParameters<T>>());
  params.insert(params.begin() + kCarParamsIndex,
                std::make_unique<SimpleCarConfig<T>>());
  return std::make_unique<systems::Parameters<T>>(std::move(params));
}

template <typename T>
void IdmController<T>::SetDefaultParameters(
    const systems::LeafContext<T>& context,
    systems::Parameters<T>* params) const {
  // Set the default IDM parameters.
  auto idm_params = dynamic_cast<IdmPlannerParameters<T>*>(
      params->get_mutable_numeric_parameter(0));
  IdmPlanner<T>::SetDefaultParameters(idm_params);

  // Set the default SimpleCar parameters (for max_acceleration).
  auto car_params = dynamic_cast<SimpleCarConfig<T>*>(
      params->get_mutable_numeric_parameter(kCarParamsIndex));
  SimpleCar<T>::SetDefaultParameters(car_params);
}

// These instantiations must match the API documentation in idm_controller.h.
template class IdmController<double>;

}  // namespace automotive
}  // namespace drake
