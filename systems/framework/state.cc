#include "drake/systems/framework/state.h"

namespace drake {
namespace systems {

template <typename T>
State<T>::State()
    : abstract_state_(std::make_unique<AbstractValues>()),
      continuous_state_(std::make_unique<ContinuousState<T>>()),
      discrete_state_(std::make_unique<DiscreteValues<T>>()) {}

template <typename T>
State<T>::~State() {}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::State)
