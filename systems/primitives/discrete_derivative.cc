#include "drake/systems/primitives/discrete_derivative.h"

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace systems {

using Eigen::MatrixXd;

template <typename T>
DiscreteDerivative<T>::DiscreteDerivative(int num_inputs, double time_step)
    : LeafSystem<T>(SystemTypeTag<systems::DiscreteDerivative>{}),
      n_(num_inputs),
      time_step_(time_step) {
  DRAKE_DEMAND(n_ > 0);
  DRAKE_DEMAND(time_step_ > 0.0);

  this->DeclareVectorInputPort("u", systems::BasicVector<T>(n_));
  this->DeclareVectorOutputPort("dudt", systems::BasicVector<T>(n_),
                                &DiscreteDerivative<T>::CalcOutput,
                                {this->xd_ticket()});

  // TODO(sherm): Prefer two state vectors of size n_ upon resolution of #9705.
  this->DeclareDiscreteState(2 * n_);
  this->DeclarePeriodicDiscreteUpdate(time_step_);
}

template <typename T>
void DiscreteDerivative<T>::set_input_history(
    drake::systems::State<T>* state,
    const Eigen::Ref<const drake::VectorX<T>>& u_n,
    const Eigen::Ref<const drake::VectorX<T>>& u_n_minus_1) const {
  DRAKE_DEMAND(u_n.size() == n_);
  DRAKE_DEMAND(u_n_minus_1.size() == n_);

  state->get_mutable_discrete_state().get_mutable_vector().get_mutable_value()
      << u_n,
      u_n_minus_1;
}

template <typename T>
void DiscreteDerivative<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>&,
    drake::systems::DiscreteValues<T>* discrete_state) const {
  // x₀[n+1] = u[n].
  discrete_state->get_mutable_vector().get_mutable_value().head(n_) =
      get_input_port().Eval(context);

  // x₁[n+1] = x₀[n].
  discrete_state->get_mutable_vector().get_mutable_value().tail(n_) =
      context.get_discrete_state(0).get_value().head(n_);
}

template <typename T>
void DiscreteDerivative<T>::CalcOutput(
    const drake::systems::Context<T>& context,
    drake::systems::BasicVector<T>* output_vector) const {
  const auto x0 = context.get_discrete_state(0).get_value().head(n_);
  const auto x1 = context.get_discrete_state(0).get_value().tail(n_);

  // y(t) = (x₀[n]-x₁[n])/h.
  output_vector->SetFromVector((x0 - x1) / time_step_);
}

template <typename T>
StateInterpolatorWithDiscreteDerivative<
    T>::StateInterpolatorWithDiscreteDerivative(int num_positions,
                                                double time_step) {
  DiagramBuilder<T> builder;

  auto pass_through = builder.template AddSystem<PassThrough>(num_positions);
  derivative_ =
      builder.template AddSystem<DiscreteDerivative>(num_positions, time_step);
  auto mux = builder.template AddSystem<Multiplexer>(
      std::vector<int>{num_positions, num_positions});

  builder.ExportInput(pass_through->get_input_port(), "position");
  builder.Connect(pass_through->get_output_port(),
                  derivative_->get_input_port());
  builder.Connect(pass_through->get_output_port(), mux->get_input_port(0));
  builder.Connect(derivative_->get_output_port(), mux->get_input_port(1));
  builder.ExportOutput(mux->get_output_port(0), "state");

  builder.BuildInto(this);
}

template <typename T>
void StateInterpolatorWithDiscreteDerivative<T>::set_initial_position(
    systems::State<T>* state,
    const Eigen::Ref<const VectorX<T>>& position) const {
  derivative_->set_input_history(
      &this->GetMutableSubsystemState(*derivative_, state), position, position);
}

template <typename T>
void StateInterpolatorWithDiscreteDerivative<T>::set_initial_state(
    systems::State<T>* state, const Eigen::Ref<const VectorX<T>>& position,
    const Eigen::Ref<const VectorX<T>>& velocity) const {
  // The derivative block implements y(t) = (u[n]-u[n-1])/h, so we want
  // u[n] = position, u[n-1] = u[n] - h*velocity
  derivative_->set_input_history(
      &this->GetMutableSubsystemState(*derivative_, state), position,
      position - derivative_->time_step() * velocity);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiscreteDerivative)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::StateInterpolatorWithDiscreteDerivative)
