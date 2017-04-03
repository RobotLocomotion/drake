#include "drake/automotive/idm_controller.h"

#include <limits>
#include <utility>
#include <vector>

#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_formula.h"
#include "drake/math/saturate.h"

namespace drake {
namespace automotive {

using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using maliput::api::Rotation;
using math::saturate;
using pose_selector::RoadOdometry;
using systems::rendering::FrameVelocity;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

static constexpr int kIdmParamsIndex{0};

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

  this->DeclareNumericParameter(IdmPlannerParameters<T>());
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
                   driving_command);
}

template <typename T>
void IdmController<T>::ImplDoCalcOutput(
    const PoseVector<T>& ego_pose, const FrameVelocity<T>& ego_velocity,
    const PoseBundle<T>& traffic_poses,
    const IdmPlannerParameters<T>& idm_params,
    DrivingCommand<T>* command) const {
  DRAKE_DEMAND(idm_params.IsValid());

  // Find the single closest car ahead.
  const RoadOdometry<T>& lead_car_odom =
      pose_selector::FindClosestLeading(road_, ego_pose, traffic_poses);
  const RoadPosition ego_position =
      pose_selector::CalcRoadPosition(road_, ego_pose.get_isometry());

  const T& s_ego = ego_position.pos.s;
  const T& s_dot_ego =
      pose_selector::GetSVelocity(RoadOdometry<double>(ego_position,
                                                       ego_velocity));
  const T& s_lead = lead_car_odom.pos.s;
  const T& s_dot_lead = pose_selector::GetSVelocity(lead_car_odom);

  // Saturate the net_distance at distance_lower_bound away from the ego car to
  // avoid near-singular solutions inherent to the IDM equation.
  const T net_distance = saturate(s_lead - s_ego - idm_params.bloat_diameter(),
                                  idm_params.distance_lower_limit(),
                                  std::numeric_limits<T>::infinity());
  const T closing_velocity = s_dot_ego - s_dot_lead;

  // Compute the acceleration command from the IDM equation.
  const T command_acceleration = IdmPlanner<T>::Evaluate(
      idm_params, s_dot_ego, net_distance, closing_velocity);
  command->set_acceleration(command_acceleration);
  command->set_steering_angle(0.);
}

// These instantiations must match the API documentation in idm_controller.h.
template class IdmController<double>;

}  // namespace automotive
}  // namespace drake
