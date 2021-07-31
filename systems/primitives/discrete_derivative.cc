#include "drake/systems/primitives/discrete_derivative.h"

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/multiplexer.h"

namespace drake {
namespace systems {

using Eigen::MatrixXd;

template <typename T>
DiscreteDerivative<T>::DiscreteDerivative(int num_inputs, double time_step,
                                          bool suppress_initial_transient)
    : LeafSystem<T>(SystemTypeTag<DiscreteDerivative>{}),
      n_(num_inputs),
      time_step_(time_step),
      suppress_initial_transient_(suppress_initial_transient) {
  DRAKE_DEMAND(n_ > 0);
  DRAKE_DEMAND(time_step_ > 0.0);

  this->DeclareVectorInputPort("u", n_);
  this->DeclareVectorOutputPort("dudt", n_, &DiscreteDerivative<T>::CalcOutput,
                                {this->xd_ticket()});
  this->DeclareDiscreteState(n_);  // u[n]
  this->DeclareDiscreteState(n_);  // u[n-1]
  if (suppress_initial_transient) {
    // An update counter, matching the value of "n" iff the simulation begins
    // at time == 0.0.
    this->DeclareDiscreteState(1);
  }
  this->DeclarePeriodicDiscreteUpdate(time_step_);
}

template <typename T>
bool DiscreteDerivative<T>::suppress_initial_transient() const {
  return suppress_initial_transient_;
}

template <typename T>
void DiscreteDerivative<T>::set_input_history(
    drake::systems::State<T>* state,
    const Eigen::Ref<const drake::VectorX<T>>& u_n,
    const Eigen::Ref<const drake::VectorX<T>>& u_n_minus_1) const {
  DRAKE_DEMAND(u_n.size() == n_);
  DRAKE_DEMAND(u_n_minus_1.size() == n_);

  state->get_mutable_discrete_state(0).SetFromVector(u_n);
  state->get_mutable_discrete_state(1).SetFromVector(u_n_minus_1);
  if (suppress_initial_transient_) {
    // Disable the transient suppression.
    state->get_mutable_discrete_state(2)[0] = 2.0;
  }
}

template <typename T>
void DiscreteDerivative<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>&,
    drake::systems::DiscreteValues<T>* state) const {
  // x₀[n+1] = u[n].
  state->set_value(0, this->get_input_port().Eval(context));

  // x₁[n+1] = x₀[n].
  state->get_mutable_vector(1).SetFrom(context.get_discrete_state(0));

  // x₂[n+i] = x₂[n] + 1
  if (suppress_initial_transient_) {
    state->get_mutable_vector(2)[0] = context.get_discrete_state(2)[0] + 1.0;
  }
}

namespace {
template <typename T>
VectorX<T> if_then_else_vector(
  const boolean<T>& f_cond,
  const Eigen::Ref<const VectorX<T>>& v_then,
  const Eigen::Ref<const VectorX<T>>& v_else) {
  DRAKE_DEMAND(v_then.size() == v_else.size());
  VectorX<T> result(v_then.size());
  for (int i = 0; i < result.size(); ++i) {
    result[i] = if_then_else(f_cond, v_then[i], v_else[i]);
  }
  return result;
}
}  // namespace

template <typename T>
void DiscreteDerivative<T>::CalcOutput(
    const drake::systems::Context<T>& context,
    drake::systems::BasicVector<T>* output_vector) const {
  const auto& x0 = context.get_discrete_state(0).get_value();
  const auto& x1 = context.get_discrete_state(1).get_value();

  // y(t) = (x₀[n]-x₁[n])/h
  const auto& derivative = ((x0 - x1) / time_step_).eval();
  if (!suppress_initial_transient_) {
    output_vector->SetFromVector(derivative);
  } else {
    const boolean<T> is_active = (context.get_discrete_state(2)[0] >= 2.0);
    output_vector->SetFromVector(if_then_else_vector<T>(
        is_active, derivative, VectorX<T>::Zero(n_)));
  }
}

template <typename T>
StateInterpolatorWithDiscreteDerivative<
    T>::StateInterpolatorWithDiscreteDerivative(
    int num_positions, double time_step, bool suppress_initial_transient) {
  DiagramBuilder<T> builder;

  derivative_ = builder.template AddSystem<DiscreteDerivative>(
      num_positions, time_step, suppress_initial_transient);
  auto mux = builder.template AddSystem<Multiplexer>(
      std::vector<int>{num_positions, num_positions});

  const auto port_index = builder.ExportInput(derivative_->get_input_port(),
                                              "position");
  builder.ConnectInput(port_index, mux->get_input_port(0));
  builder.Connect(derivative_->get_output_port(), mux->get_input_port(1));
  builder.ExportOutput(mux->get_output_port(0), "state");

  builder.BuildInto(this);
}

template <typename T>
bool StateInterpolatorWithDiscreteDerivative<
      T>::suppress_initial_transient() const {
  return derivative_->suppress_initial_transient();
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
