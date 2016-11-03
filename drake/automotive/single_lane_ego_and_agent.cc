#include "drake/automotive/single_lane_ego_and_agent.h"

#include "drake/common/drake_export.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace automotive {

template <typename T>
SingleLaneEgoAndAgent<T>::SingleLaneEgoAndAgent(const T& v_0, const T& a_agent)
    : systems::Diagram<T>() {
  // The reference velocity must be strictly positive.
  DRAKE_ASSERT(v_0 > 0);

  systems::DiagramBuilder<T> builder;

  // LinearCar systems have a state vector.
  agent_car_ = builder.AddSystem(std::make_unique<LinearCar<T>>());
  ego_car_ = builder.AddSystem(std::make_unique<LinearCar<T>>());

  value_ = builder.AddSystem(std::make_unique<systems::ConstantVectorSource<T>>(
      a_agent /* acceleration of the agent */));
  planner_ = builder.AddSystem(std::make_unique<IdmPlanner<T>>(
      v_0 /* desired velocity of the ego car */));

  builder.Connect(*planner_, *ego_car_);
  builder.Connect(*value_, *agent_car_);
  builder.Connect(ego_car_->get_output_port(), planner_->get_input_port(0));
  builder.Connect(agent_car_->get_output_port(), planner_->get_input_port(1));

  builder.ExportOutput(ego_car_->get_output_port());
  builder.ExportOutput(agent_car_->get_output_port());

  builder.BuildInto(this);
}

template <typename T>
bool SingleLaneEgoAndAgent<T>::has_any_direct_feedthrough() const {
  return false;
}

template <typename T>
void SingleLaneEgoAndAgent<T>::SetDefaultState(
    systems::Context<T>* context) const {
  // Obtain mutable references to the contexts for each car.
  DRAKE_DEMAND(context != nullptr);
  systems::Context<T>* context_ego =
      this->GetMutableSubsystemContext(context, ego_car_);
  systems::ContinuousState<T>* x_ego =
      context_ego->get_mutable_continuous_state();
  DRAKE_DEMAND(x_ego != nullptr);
  systems::Context<T>* context_agent =
      this->GetMutableSubsystemContext(context, agent_car_);
  systems::ContinuousState<T>* x_agent =
      context_agent->get_mutable_continuous_state();
  DRAKE_DEMAND(x_agent != nullptr);

  // The default state vector contains all zeros.
  x_ego->get_mutable_vector()->SetAtIndex(0, 0.0);
  x_ego->get_mutable_vector()->SetAtIndex(1, 0.0);
  x_agent->get_mutable_vector()->SetAtIndex(0, 0.0);
  x_agent->get_mutable_vector()->SetAtIndex(1, 0.0);
}

template class DRAKE_EXPORT SingleLaneEgoAndAgent<double>;
template class DRAKE_EXPORT SingleLaneEgoAndAgent<AutoDiffXd>;

}  // namespace automotive
}  // namespace drake
