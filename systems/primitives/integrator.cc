#include "drake/systems/primitives/integrator.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"

namespace drake {
namespace systems {

template <typename T>
Integrator<T>::Integrator(const VectorX<double>& initial_value)
    : VectorSystem<T>(SystemTypeTag<Integrator>{}, initial_value.size(),
                      initial_value.size(),
                      /* direct_feedthrough = */ false) {
  initial_value_ = initial_value;
  const BasicVector<T> basic_vector(initial_value_);
  this->DeclareContinuousState(basic_vector);
}

template <typename T>
template <typename U>
Integrator<T>::Integrator(const Integrator<U>& other)
    : Integrator<T>(other.initial_value_) {}

template <typename T>
Integrator<T>::~Integrator() = default;

template <typename T>
void Integrator<T>::set_integral_value(
    Context<T>* context, const Eigen::Ref<const VectorX<T>>& value) const {
  this->ValidateContext(context);
  VectorBase<T>& state_vector = context->get_mutable_continuous_state_vector();
  DRAKE_THROW_UNLESS(value.size() == state_vector.size());
  state_vector.SetFromVector(value);
}

template <typename T>
void Integrator<T>::set_default_integral_value(
    const VectorX<double>& initial_value) {
  DRAKE_THROW_UNLESS(initial_value.size() == initial_value_.size());
  initial_value_ = initial_value;
}

template <typename T>
void Integrator<T>::DoCalcVectorTimeDerivatives(
    const Context<T>&, const Eigen::VectorBlock<const VectorX<T>>& input,
    const Eigen::VectorBlock<const VectorX<T>>& state,
    Eigen::VectorBlock<VectorX<T>>* derivatives) const {
  unused(state);
  *derivatives = input;
}

template <typename T>
void Integrator<T>::DoCalcVectorOutput(
    const Context<T>&, const Eigen::VectorBlock<const VectorX<T>>& input,
    const Eigen::VectorBlock<const VectorX<T>>& state,
    Eigen::VectorBlock<VectorX<T>>* output) const {
  unused(input);
  // TODO(david-german-tri): Remove this copy by allowing output ports to be
  // mere pointers to state variables (or cache lines).
  *output = state;
}

template <typename T>
void Integrator<T>::SetDefaultState(const Context<T>&, State<T>* state) const {
  ContinuousState<T>& continuous_state = state->get_mutable_continuous_state();
  DRAKE_DEMAND(initial_value_.size() == continuous_state.size());
  continuous_state.SetFromVector(initial_value_.template cast<T>());
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Integrator);
