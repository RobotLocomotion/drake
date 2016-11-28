#include "drake/automotive/single_lane_ego_and_agent.h"

#include "drake/common/symbolic_formula.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace automotive {

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
  planner_ = builder.AddSystem(std::make_unique<IdmPlanner<T>>(
      v_ref /* desired velocity of the ego car */));

  builder.Connect(*planner_, *ego_car_);
  builder.Connect(*value, *agent_car_);
  builder.Connect(ego_car_->get_output_port(), planner_->get_ego_port());
  builder.Connect(agent_car_->get_output_port(), planner_->get_agent_port());

  builder.ExportOutput(ego_car_->get_output_port());    // Exports to port 0.
  builder.ExportOutput(agent_car_->get_output_port());  // Exports to port 1.

  builder.BuildInto(this);
}

template <typename T>
void SingleLaneEgoAndAgent<T>::SetDefaultState(
    systems::Context<T>* context) const {
  // Obtain mutable references to the contexts for each car.
  DRAKE_DEMAND(context != nullptr);
  systems::Context<T>* context_ego =
      this->GetMutableSubsystemContext(context, ego_car_);
  DRAKE_DEMAND(context_ego != nullptr);
  systems::Context<T>* context_agent =
      this->GetMutableSubsystemContext(context, agent_car_);
  DRAKE_DEMAND(context_agent != nullptr);

  // Set the default state for both cars.
  ego_car_->SetDefaultState(context_ego);
  agent_car_->SetDefaultState(context_agent);
}

// These instantiations must match the API documentation in
// single_lane_ego_and_agent.h.
template class SingleLaneEgoAndAgent<double>;
template class SingleLaneEgoAndAgent<AutoDiffXd>;
template class SingleLaneEgoAndAgent<symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
