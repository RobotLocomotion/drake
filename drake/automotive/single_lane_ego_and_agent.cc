#include "drake/automotive/single_lane_ego_and_agent.h"

#include <utility>

#include "drake/common/symbolic_formula.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace automotive {

template <typename T>
Idm<T>::Idm() {
  const int kEgoCarOutputVectorSize = 2;
  const int kAgentCarOutputVectorSize = 2;
  const int kLinearAccelerationSize = 1;
  // Declare the ego car input port.
  this->DeclareInputPort(systems::kVectorValued, kEgoCarOutputVectorSize);
  // Declare the agent car input port.
  this->DeclareInputPort(systems::kVectorValued, kAgentCarOutputVectorSize);
  // Declare the output port.
  this->DeclareOutputPort(systems::kVectorValued, kLinearAccelerationSize);
}

template <typename T>
Idm<T>::~Idm() {}

template <typename T>
const systems::InputPortDescriptor<T>& Idm<T>::get_ego_port() const {
  return systems::System<T>::get_input_port(0);
}

template <typename T>
const systems::InputPortDescriptor<T>& Idm<T>::get_agent_port() const {
  return systems::System<T>::get_input_port(1);
}

template <typename T>
void Idm<T>::DoCalcOutput(const systems::Context<T>& context,
                          systems::SystemOutput<T>* output) const {
  const T l_car = 4.5;  // Length of the lead car [m].

  // Obtain the input/output data structures.
  const systems::BasicVector<T>* input_ego =
      this->EvalVectorInput(context, this->get_ego_port().get_index());
  const systems::BasicVector<T>* input_agent =
      this->EvalVectorInput(context, this->get_agent_port().get_index());
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector != nullptr);

  // Parse the inputs.
  const T& x_ego = input_ego->GetAtIndex(0);
  const T& v_ego = input_ego->GetAtIndex(1);
  const T& x_agent = input_agent->GetAtIndex(0);
  const T& v_agent = input_agent->GetAtIndex(1);

  // Obtain the parameters.
  const int kParamsIndex = 0;
  const IdmPlannerParameters<T>& params =
      this->template GetNumericParameter<IdmPlannerParameters>(context,
                                                               kParamsIndex);
  // Evaluate the IDM equation.
  output_vector->SetAtIndex(
      0, IdmPlanner<T>::Evaluate(params, v_ego, x_agent - x_ego - l_car,
                                 v_agent - v_ego));
}

template <typename T>
std::unique_ptr<systems::Parameters<T>> Idm<T>::AllocateParameters() const {
  auto params = std::make_unique<IdmPlannerParameters<T>>();
  return std::make_unique<systems::Parameters<T>>(std::move(params));
}

template <typename T>
void Idm<T>::SetDefaultParameters(const systems::LeafContext<T>& context,
                                  systems::Parameters<T>* params) const {
  auto idm_params = dynamic_cast<IdmPlannerParameters<T>*>(
      params->get_mutable_numeric_parameter(0));
  IdmPlanner<T>::SetDefaultParameters(idm_params);
}

template <typename T>
SingleLaneEgoAndAgent<T>::SingleLaneEgoAndAgent(
    const T& x_ego_init, const T& v_ego_init, const T& x_agent_init,
    const T& v_agent_init, const T& v_ref, const T& a_agent)
    : systems::Diagram<T>() {
  // The reference velocity must be strictly positive.
  DRAKE_DEMAND(v_ref > 0);

  systems::DiagramBuilder<T> builder;

  // Instantiate LinearCar systems at some initial positions and velocities.
  agent_car_ = builder.AddSystem(
      std::make_unique<LinearCar<T>>(x_agent_init, v_agent_init));
  ego_car_ =
      builder.AddSystem(std::make_unique<LinearCar<T>>(x_ego_init, v_ego_init));

  // Instantiate additional subsystems feed the two cars their inputs.
  const systems::ConstantVectorSource<T>* value =
      builder.AddSystem(std::make_unique<systems::ConstantVectorSource<T>>(
          a_agent /* acceleration of the agent */));
  planner_ = builder.AddSystem(std::make_unique<Idm<T>>());

  builder.Connect(*planner_, *ego_car_);
  builder.Connect(*value, *agent_car_);
  builder.Connect(ego_car_->get_output_port(), planner_->get_ego_port());
  builder.Connect(agent_car_->get_output_port(), planner_->get_agent_port());

  builder.ExportOutput(ego_car_->get_output_port());    // Exports to port 0.
  builder.ExportOutput(agent_car_->get_output_port());  // Exports to port 1.

  builder.BuildInto(this);
}

// These instantiations must match the API documentation in
// single_lane_ego_and_agent.h.
template class SingleLaneEgoAndAgent<double>;
template class SingleLaneEgoAndAgent<AutoDiffXd>;
template class SingleLaneEgoAndAgent<symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
