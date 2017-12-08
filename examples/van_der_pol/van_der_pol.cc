#include "drake/examples/van_der_pol/van_der_pol.h"

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/system_constraint.h"

namespace drake {
namespace examples {
namespace van_der_pol {

template <typename T>
VanDerPolOscillator<T>::VanDerPolOscillator() : systems::LeafSystem<T>() {
  // State is (q,q̇).
  this->DeclareContinuousState(1, 1, 0);

  // First output, y₁ = q, for interesting estimation problems.
  this->DeclareVectorOutputPort(systems::BasicVector<T>(1),
                                &VanDerPolOscillator::CopyPositionToOutput);

  // Second output, y₂ = [q,q̇]', for e.g. visualizing the full state.
  this->DeclareVectorOutputPort(systems::BasicVector<T>(2),
                                &VanDerPolOscillator::CopyFullStateToOutput);

  // Single parameter, μ, with default μ=1.
  this->DeclareNumericParameter(systems::BasicVector<T>(Vector1<T>(1.0)));

  // Declare μ≥0 constraint.
  typename systems::SystemConstraint<T>::CalcCallback mu = [](
      const systems::Context<T>& context, VectorX<T>* value) {
    // Extract μ from the parameters.
    *value = Vector1<T>(context.get_numeric_parameter(0).GetAtIndex(0));
  };
  this->DeclareInequalityConstraint(mu, 1, "mu ≥ 0");
}

// q̈ + μ(q² - 1)q̇ + q = 0
template <typename T>
void VanDerPolOscillator<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const T q =
      context.get_continuous_state().get_generalized_position().GetAtIndex(0);
  const T qdot =
      context.get_continuous_state().get_generalized_velocity().GetAtIndex(0);
  const T mu = context.get_numeric_parameter(0).GetAtIndex(0);

  using std::pow;
  const T qddot = -mu * (q * q - 1) * qdot - q;

  derivatives->get_mutable_generalized_position().SetAtIndex(0, qdot);
  derivatives->get_mutable_generalized_velocity().SetAtIndex(0, qddot);
}

template <typename T>
void VanDerPolOscillator<T>::CopyPositionToOutput(
    const systems::Context<T>& context, systems::BasicVector<T>* output) const {
  output->SetAtIndex(
      0,
      context.get_continuous_state().get_generalized_position().GetAtIndex(0));
}

template <typename T>
void VanDerPolOscillator<T>::CopyFullStateToOutput(
    const systems::Context<T>& context, systems::BasicVector<T>* output) const {
  output->SetFromVector(context.get_continuous_state_vector().CopyToVector());
}

}  // namespace van_der_pol
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::van_der_pol::VanDerPolOscillator)
