#include "drake/systems/primitives/discrete_derivative.h"

#include <memory>
#include <vector>

#include "drake/common/default_scalars.h"

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
                                &DiscreteDerivative<T>::CalcOutput);

  // TODO(sherm): Prefer two state vectors of size n_ upon resolution of #9705.
  this->DeclareDiscreteState(2 * n_);
  this->DeclarePeriodicDiscreteUpdate(time_step_);
}

template <typename T>
void DiscreteDerivative<T>::set_state(
    const Eigen::Ref<const drake::VectorX<T>>& u,
    drake::systems::Context<T>* context) const {
  DRAKE_DEMAND(u.size() == n_);

  context->get_mutable_discrete_state_vector().get_mutable_value() << u, u;
}

template <typename T>
void DiscreteDerivative<T>::DoCalcDiscreteVariableUpdates(
    const drake::systems::Context<T>& context,
    const std::vector<const drake::systems::DiscreteUpdateEvent<T>*>&,
    drake::systems::DiscreteValues<T>* discrete_state) const {
  // x₀[n+1] = u[n].
  VectorX<T> u = this->EvalEigenVectorInput(context, 0);
  discrete_state->get_mutable_vector().get_mutable_value().head(n_) =
      this->EvalEigenVectorInput(context, 0);
discrete_state->get_mutable_vector().get_mutable_value().head(n_) = u;

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

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiscreteDerivative)
