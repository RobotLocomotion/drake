#include "drake/systems/primitives/discrete_time_integrator.h"

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"
#include "drake/common/unused.h"

namespace drake {
namespace systems {

template <typename T>
DiscreteTimeIntegrator<T>::DiscreteTimeIntegrator(int size, double time_step)
    : LeafSystem<T>(SystemTypeTag<DiscreteTimeIntegrator>{}),
      time_step_{time_step} {
  DRAKE_THROW_UNLESS(size > 0);
  DRAKE_THROW_UNLESS(time_step > 0);
  this->DeclareVectorInputPort("u", size);
  auto state_index = this->DeclareDiscreteState(size);
  this->DeclarePeriodicDiscreteUpdateEvent(time_step, 0.0,
                                           &DiscreteTimeIntegrator::Update);
  this->DeclareStateOutputPort("y", state_index);
}

template <typename T>
template <typename U>
DiscreteTimeIntegrator<T>::DiscreteTimeIntegrator(
    const DiscreteTimeIntegrator<U>& other)
    : DiscreteTimeIntegrator<T>(other.get_input_port().size(),
                                other.time_step()) {}

template <typename T>
DiscreteTimeIntegrator<T>::~DiscreteTimeIntegrator() = default;

template <typename T>
void DiscreteTimeIntegrator<T>::set_integral_value(
    Context<T>* context, const Eigen::Ref<const VectorX<T>>& value) const {
  this->ValidateContext(context);
  BasicVector<T>& state_vector = context->get_mutable_discrete_state_vector();
  // Asserts that the input value is a column vector of the appropriate size.
  DRAKE_THROW_UNLESS(value.rows() == state_vector.size());
  state_vector.SetFromVector(value);
}

template <typename T>
void DiscreteTimeIntegrator<T>::Update(const Context<T>& context,
                                       DiscreteValues<T>* next_state) const {
  const VectorX<T>& state = context.get_discrete_state_vector().value();
  next_state->set_value(state +
                        time_step_ * this->get_input_port().Eval(context));
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiscreteTimeIntegrator);
