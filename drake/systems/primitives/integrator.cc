#include "drake/systems/primitives/integrator.h"

#include <stdexcept>
#include <string>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {

template <typename T>
Integrator<T>::Integrator(int size) : SisoVectorSystem<T>(size, size) {
  this->DeclareContinuousState(size);
}

template <typename T>
Integrator<T>::~Integrator() {}

template <typename T>
void Integrator<T>::set_integral_value(
    Context<T>* context, const Eigen::Ref<const VectorX<T>>& value) const {
  VectorBase<T>* state_vector = context->get_mutable_continuous_state_vector();
  // Asserts that the input value is a column vector of the appropriate size.
  DRAKE_ASSERT(value.rows() == state_vector->size() && value.cols() == 1);
  state_vector->SetFromVector(value);
}

template <typename T>
void Integrator<T>::DoCalcVectorTimeDerivatives(
    const Context<T>& context,
    const Eigen::VectorBlock<const VectorX<T>>& input,
    const Eigen::VectorBlock<const VectorX<T>>& state,
    Eigen::VectorBlock<VectorX<T>>* derivatives) const {
  *derivatives = input;
}

template <typename T>
void Integrator<T>::DoCalcVectorOutput(
    const Context<T>& context,
    const Eigen::VectorBlock<const VectorX<T>>& input,
    const Eigen::VectorBlock<const VectorX<T>>& state,
    Eigen::VectorBlock<VectorX<T>>* output) const {
  // TODO(david-german-tri): Remove this copy by allowing output ports to be
  // mere pointers to state variables (or cache lines).
  *output = state;
}

template <typename T>
Integrator<AutoDiffXd>* Integrator<T>::DoToAutoDiffXd() const {
  return new Integrator<AutoDiffXd>(this->get_input_port().size());
}

template <typename T>
Integrator<symbolic::Expression>* Integrator<T>::DoToSymbolic() const {
  return new Integrator<symbolic::Expression>(this->get_input_port().size());
}

// Explicitly instantiates on the most common scalar types.
template class Integrator<double>;
template class Integrator<AutoDiffXd>;
template class Integrator<symbolic::Expression>;

}  // namespace systems
}  // namespace drake
