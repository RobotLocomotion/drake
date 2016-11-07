#include "drake/automotive/single_lane_ego_and_agent.h"

#include "drake/common/drake_export.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace automotive {

template <typename T>
SingleLaneEgoAndAgent<T>::SingleLaneEgoAndAgent(const T& v_0, const T& a_agent)
  : systems::Diagram<T>() {

  // Keep the reference velocity nonnegative.
  DRAKE_ASSERT(v_0 >= 0);

  systems::DiagramBuilder<T> builder;
  integrator_ = builder.AddSystem(std::make_unique<systems::Integrator<T>>(2));
  //ego_car_ = builder.AddSystem(std::make_unique<LinearCar<T>>());
  //agent_car_ = builder.AddSystem(std::make_unique<LinearCar<T>>());
  //planner_ = builder.AddSystem(std::make_unique<IdmPlanner<T>>(
  //                                            v_0 /* desired velocity */));
  //value_ = builder.AddSystem(std::make_unique<systems::ConstantVectorSource<T>>(
  //                                  a_agent /* agent car acceleration */));

  // Create an output port for each car.
  builder.ExportOutput(integrator_->get_output_port(0));
  //builder.ExportOutput(ego_car_->get_output_port());
  //builder.ExportOutput(agent_car_->get_output_port());

  //builder.Connect(*planner_, *ego_car_);
  //builder.Connect(*value_, *agent_car_);
  //builder.Connect(ego_car_->get_output_port(),
  //                planner_->get_input_port(0));
  //builder.Connect(agent_car_->get_output_port(),
  //                planner_->get_input_port(1));
  builder.BuildInto(this);
}

template <typename T>
const systems::SystemPortDescriptor<T>&
SingleLaneEgoAndAgent<T>::get_ego_car_output_port() const {
  return systems::System<T>::get_output_port(0);
}

template <typename T>
const systems::SystemPortDescriptor<T>&
SingleLaneEgoAndAgent<T>::get_agent_car_output_port() const {
  return systems::System<T>::get_output_port(1);
}

template <typename T>
void SingleLaneEgoAndAgent<T>::SetDefaultState(
    systems::Context<T>* context) const {
  DRAKE_DEMAND(context != nullptr);
  systems::ContinuousState<T>* xc = context->get_mutable_continuous_state();
  DRAKE_DEMAND(xc != nullptr);

  // Write the zero configuration into the continuous state.
  VectorX<T> x0 = VectorX<T>::Zero(2);
  xc->SetFromVector(x0);
}

template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
SingleLaneEgoAndAgent<T>::AllocateContinuousState() const {
  auto state = std::make_unique<SingleLaneEgoAndAgentState<T>>();
  // Define the initial state values.
  state->set_x_e(T{0.0});
  state->set_v_e(T{0.0});
  state->set_x_a(T{0.0});
  state->set_v_a(T{0.0});
  return std::make_unique<systems::ContinuousState<T>>(std::move(state));
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>>
SingleLaneEgoAndAgent<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  return std::make_unique<SingleLaneEgoAndAgentState<T>>();
}

template class DRAKE_EXPORT SingleLaneEgoAndAgent<double>;
template class DRAKE_EXPORT SingleLaneEgoAndAgent<AutoDiffXd>;

}  // namespace automotive
}  // namespace drake
