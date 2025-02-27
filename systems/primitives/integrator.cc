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
  const BasicVector<T> basic_vector(initial_value_.template cast<T>());
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
  // Asserts that the input value is a column vector of the appropriate size.
  DRAKE_DEMAND(value.rows() == state_vector.size() && value.cols() == 1);
  state_vector.SetFromVector(value);
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

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Integrator);
