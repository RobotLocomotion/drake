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

  ego_car_ = builder.AddSystem(std::make_unique<LinearCar<T>>());
  agent_car_ = builder.AddSystem(std::make_unique<LinearCar<T>>());
  //agent_car_
  //  = builder.AddSystem(std::make_unique<systems::Integrator<T>>(size));
  value_
    = builder.AddSystem(std::make_unique<systems::ConstantVectorSource<T>>(
        a_agent /* acceleration of the agent */ ));
  planner_ = builder.AddSystem(std::make_unique<IdmPlanner<T>>(
        v_0 /* desired velocity of the ego car */ ));

  builder.Connect(*planner_, *ego_car_);
  builder.Connect(*value_, *agent_car_);
  builder.Connect(ego_car_->get_output_port(),
                  planner_->get_input_port(0));
  builder.Connect(agent_car_->get_output_port(),
                  planner_->get_input_port(1));

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
  DRAKE_DEMAND(context != nullptr);
  systems::ContinuousState<T>* xc = context->get_mutable_continuous_state();
  DRAKE_DEMAND(xc != nullptr);

  // Write the zero configuration into the continuous state.
  VectorX<T> x0 = VectorX<T>::Zero(4);
  xc->SetFromVector(x0);
}

template class DRAKE_EXPORT SingleLaneEgoAndAgent<double>;
template class DRAKE_EXPORT SingleLaneEgoAndAgent<AutoDiffXd>;

}  // namespace automotive
}  // namespace drake
