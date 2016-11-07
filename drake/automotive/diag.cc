#include "drake/automotive/diag.h"

#include "drake/common/drake_export.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/framework/diagram_builder.h"

using std::make_unique;

namespace drake {
namespace automotive {

template <typename T>
Diag<T>::Diag(const T& Kp, const T& Ki, const T& Kd,
              const T& v_0, const T& a_agent,
              int size)
    : Diag(VectorX<T>::Ones(size) * Kp, VectorX<T>::Ones(size) * Ki,
           VectorX<T>::Ones(size) * Kd,
           v_0, a_agent
           ) { }

template <typename T>
Diag<T>::Diag(const VectorX<T>& Kp, const VectorX<T>& Ki,
              const VectorX<T>& Kd,
              const T& v_0, const T& a_agent
              ) :
    systems::Diagram<T>() {
  const int size = Kp.size();
  DRAKE_ASSERT(size > 0);

  DRAKE_ASSERT(Ki.size() == size);
  DRAKE_ASSERT(Kd.size() == size);

  for (int i = 0; i < size; ++i) {
    DRAKE_ASSERT(Kp(i) >= 0);
    DRAKE_ASSERT(Ki(i) >= 0);
    DRAKE_ASSERT(Kd(i) >= 0);
  }

  systems::DiagramBuilder<T> builder;
  /*
  pass_through_ = builder.AddSystem(make_unique<systems::PassThrough<T>>(size));
  proportional_gain_ = builder.AddSystem(make_unique<systems::Gain<T>>(Kp));
  integral_gain_ = builder.AddSystem(make_unique<systems::Gain<T>>(Ki));
  derivative_gain_ = builder.AddSystem(make_unique<systems::Gain<T>>(Kd));
  another_gain_ = builder.AddSystem(make_unique<systems::Gain<T>>(Kd));
  integrator_ = builder.AddSystem(make_unique<systems::Integrator<T>>(size));
  adder_
    = builder.AddSystem(make_unique<systems::Adder<T>>(3, size));

  // Input 0 connects to the proportional and integral components.
  builder.ExportInput(pass_through_->get_input_port(0));
  // Input 1 connects directly to the derivative component.
  builder.ExportInput(derivative_gain_->get_input_port());
  builder.Connect(*pass_through_, *proportional_gain_);
  builder.Connect(*pass_through_, *integrator_);
  builder.Connect(*integrator_, *another_gain_);
  builder.Connect(*another_gain_, *integral_gain_);
  builder.Connect(proportional_gain_->get_output_port(),
                  adder_->get_input_port(0));
  builder.Connect(integral_gain_->get_output_port(),
                  adder_->get_input_port(1));
  builder.Connect(derivative_gain_->get_output_port(),
                  adder_->get_input_port(2));
  builder.ExportOutput(adder_->get_output_port());
  */

  //another_gain_ = builder.AddSystem(make_unique<systems::Gain<T>>(Kp));
  //integral_gain_ = builder.AddSystem(make_unique<systems::Gain<T>>(Ki));
  //integrator_ = builder.AddSystem(make_unique<systems::Integrator<T>>(size));

  ego_car_ = builder.AddSystem(std::make_unique<LinearCar<T>>());
  agent_car_ = builder.AddSystem(std::make_unique<LinearCar<T>>());
  //position_ego_car_integrator_
  //  = builder.AddSystem(std::make_unique<systems::Integrator<T>>(1));
  //velocity_ego_car_integrator_
  //  = builder.AddSystem(std::make_unique<systems::Integrator<T>>(1));
  //position_agent_car_integrator_
  //  = builder.AddSystem(std::make_unique<systems::Integrator<T>>(1));
  //velocity_agent_car_integrator_
  //  = builder.AddSystem(std::make_unique<systems::Integrator<T>>(1));
  //agent_car_
  //  = builder.AddSystem(std::make_unique<systems::Integrator<T>>(size));
  value_
    = builder.AddSystem(std::make_unique<systems::ConstantVectorSource<T>>(
                                    a_agent ));
  //planner_ = builder.AddSystem(std::make_unique<IdmPlannerFourInputs<T>>(v_0));
  planner_ = builder.AddSystem(std::make_unique<IdmPlanner<T>>(v_0));

  //builder.Connect(*integrator_, *another_gain_);
  //builder.Connect(*another_gain_, *integral_gain_);

  builder.Connect(*planner_, *ego_car_);
  builder.Connect(*value_, *agent_car_);
  builder.Connect(ego_car_->get_output_port(),
                  planner_->get_input_port(0));
  builder.Connect(agent_car_->get_output_port(),
                  planner_->get_input_port(1));

  //builder.Connect(*planner_, *velocity_ego_car_integrator_);
  //builder.Connect(*velocity_ego_car_integrator_,
  //                *position_ego_car_integrator_);
  //builder.Connect(*value_, *velocity_agent_car_integrator_);
  //builder.Connect(*velocity_agent_car_integrator_,
  //                *position_agent_car_integrator_);
  //builder.Connect(position_ego_car_integrator_->get_output_port(0),
  //                planner_->get_input_port(0));
  //builder.Connect(velocity_ego_car_integrator_->get_output_port(0),
  //                planner_->get_input_port(1));
  //builder.Connect(position_agent_car_integrator_->get_output_port(0),
  //                planner_->get_input_port(2));
  //builder.Connect(velocity_agent_car_integrator_->get_output_port(0),
  //                planner_->get_input_port(3));

  // Input 0 connects to the proportional and integral components.
  //builder.ExportInput(integrator_->get_input_port(0));
  builder.ExportOutput(ego_car_->get_output_port());
  builder.ExportOutput(agent_car_->get_output_port());
  //builder.ExportOutput(ego_car_->get_output_port());
  //builder.ExportOutput(agent_car_->get_output_port());

  /*
  ego_car_ = builder.AddSystem(std::make_unique<LinearCar<T>>());
  agent_car_ = builder.AddSystem(std::make_unique<LinearCar<T>>());
  planner_ = builder.AddSystem(std::make_unique<IdmPlanner<T>>(
                                              v_0 ));
  value_ = builder.AddSystem(std::make_unique<systems::ConstantVectorSource<T>>(
                                    a_agent ));

  // Create an output port for each car.
  //builder.Connect(*planner_, *ego_car_);
  builder.Connect(*value_, *agent_car_);
  //builder.Connect(ego_car_->get_output_port(),
  //                planner_->get_input_port(0));
  //builder.Connect(agent_car_->get_output_port(),
  //                planner_->get_input_port(1));
  //builder.ExportOutput(ego_car_->get_output_port());
  builder.ExportOutput(agent_car_->get_output_port());
  */
  builder.BuildInto(this);
}

template <typename T>
bool Diag<T>::has_any_direct_feedthrough() const {
  return false;
}

template <typename T>
void Diag<T>::SetDefaultState(
    systems::Context<T>* context) const {
  DRAKE_DEMAND(context != nullptr);
  systems::ContinuousState<T>* xc = context->get_mutable_continuous_state();
  DRAKE_DEMAND(xc != nullptr);

  // Write the zero configuration into the continuous state.
  VectorX<T> x0 = VectorX<T>::Zero(4);
  xc->SetFromVector(x0);
}
  /*
template <typename T>
void Diag<T>::SetDefaultState(systems::Context<T>* context) const {
  const int size = systems::Diagram<T>::get_input_port(0).get_size();
  set_integral_value(context, VectorX<T>::Zero(size));
}

template <typename T>
void Diag<T>::set_integral_value(
  systems::Context<T>* context,
  const Eigen::Ref<const VectorX<T>>& value) const {
  systems::Context<T>* integrator_context =
    systems::Diagram<T>::GetMutableSubsystemContext(context, integrator_);
  integrator_->set_integral_value(integrator_context, value);
}
  */
template class DRAKE_EXPORT Diag<double>;
template class DRAKE_EXPORT Diag<AutoDiffXd>;

}  // namespace automotive
}  // namespace drake
