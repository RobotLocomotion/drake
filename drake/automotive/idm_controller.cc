#include "drake/automotive/idm_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/pose_selector.h"
#include "drake/common/cond.h"
#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_formula.h"

namespace drake {

using maliput::api::GeoPosition;
using maliput::api::RoadGeometry;
using maliput::api::RoadPosition;
using systems::rendering::PoseBundle;
using systems::rendering::PoseVector;

namespace automotive {

template <typename T>
IdmController<T>::IdmController(const RoadGeometry* road) : road_(road) {
  // Declare the ego car input.
  this->DeclareInputPort(systems::kVectorValued, PoseVector<T>::kSize);
  // Declare the traffic car inputs.
  this->DeclareAbstractInputPort();
  // Declare the output port.
  this->DeclareOutputPort(systems::kVectorValued,
                          DrivingCommandIndices::kNumCoordinates);
}

template <typename T>
IdmController<T>::~IdmController() {}

template <typename T>
const systems::InputPortDescriptor<T>& IdmController<T>::ego_pose_input()
    const {
  return systems::System<T>::get_input_port(0);
}

template <typename T>
const systems::InputPortDescriptor<T>&
IdmController<T>::agent_pose_bundle_input() const {
  return systems::System<T>::get_input_port(1);
}

template <typename T>
void IdmController<T>::DoCalcOutput(const systems::Context<T>& context,
                                    systems::SystemOutput<T>* output) const {
  // Obtain the parameters.
  const int kParamsIndex = 0;
  const IdmPlannerParameters<T>& params =
      this->template GetNumericParameter<IdmPlannerParameters>(context,
                                                               kParamsIndex);
  // Obtain the input/output data structures.
  const PoseVector<T>* const ego_pose =
      this->template EvalVectorInput<PoseVector>(
          context, this->ego_pose_input().get_index());
  DRAKE_ASSERT(ego_pose != nullptr);

  const PoseBundle<T>* const agent_poses =
      this->template EvalInputValue<PoseBundle<T>>(
          context, this->agent_pose_bundle_input().get_index());
  DRAKE_ASSERT(agent_poses != nullptr);

  systems::BasicVector<T>* const command_output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(command_output_vector != nullptr);
  DrivingCommand<T>* const driving_command =
      dynamic_cast<DrivingCommand<T>*>(command_output_vector);
  DRAKE_ASSERT(driving_command != nullptr);

  const RoadPosition& agent_position =
      PoseSelector<T>::SelectClosestLeadingPosition(*road_, *ego_pose,
                                                    *agent_poses);
  ImplDoCalcOutput(*ego_pose, agent_position, params, driving_command);
}

template <typename T>
void IdmController<T>::ImplDoCalcOutput(const PoseVector<T>& ego_pose,
                                        const RoadPosition& agent_road_position,
                                        const IdmPlannerParameters<T>& params,
                                        DrivingCommand<T>* command) const {
  const T car_length = 4.5;

  const T& s_ego =
      PoseSelector<T>::GetRoadPosition(*road_, ego_pose.get_isometry()).pos.s;
  const T& s_dot_ego = 10.;  // TODO(jadecastro): Retrieve an actual velocity.
  const T& s_agent = agent_road_position.pos.s;
  const T& s_dot_agent = 0.;  // TODO(jadecastro): Retrieve an actual velocity.

  // Ensure that we are supplying the planner with sane parameters and input
  // values.
  const T net_distance = s_agent - s_ego - car_length;
  // ********** TODO(jadecastro): Saturate this instead.
  DRAKE_DEMAND(net_distance > 0.);
  const T closing_velocity = s_dot_ego - s_dot_agent;

  // Output the acceleration command from the IDM equation and allocate the
  // result to either the throttle or brake.
  const T command_acceleration =
      IdmPlanner<T>::Evaluate(params, s_dot_ego, net_distance,
                              closing_velocity);
  command->set_throttle(
      cond(command_acceleration < T(0.), T(0.), command_acceleration));
  command->set_brake(
      cond(command_acceleration >= T(0.), T(0.), -command_acceleration));
  command->set_steering_angle(0.);
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>> IdmController<T>::AllocateOutputVector(
    const systems::OutputPortDescriptor<T>& descriptor) const {
  DRAKE_DEMAND(descriptor.get_index() <= 1);
  return std::make_unique<DrivingCommand<T>>();
}

template <typename T>
std::unique_ptr<systems::Parameters<T>> IdmController<T>::AllocateParameters()
    const {
  auto params = std::make_unique<IdmPlannerParameters<T>>();
  return std::make_unique<systems::Parameters<T>>(std::move(params));
}

template <typename T>
void IdmController<T>::SetDefaultParameters(
    const systems::LeafContext<T>& context, systems::Parameters<T>* params)
    const {
  auto idm_params = dynamic_cast<IdmPlannerParameters<T>*>(
      params->get_mutable_numeric_parameter(0));
  IdmPlanner<T>::SetDefaultParameters(idm_params);
}

// These instantiations must match the API documentation in
// idm_planner.h.
template class IdmController<double>;
// template class IdmController<drake::TaylorVarXd>;
// template class IdmController<drake::symbolic::Expression>;
// TODO(jadecastro): Need SFNAE or some other thing here to activate the above.

}  // namespace automotive
}  // namespace drake
