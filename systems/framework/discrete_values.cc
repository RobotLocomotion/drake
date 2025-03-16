#include "drake/systems/framework/discrete_values.h"

namespace drake {
namespace systems {

template <typename T>
DiscreteValues<T>::~DiscreteValues() {}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiscreteValues);
