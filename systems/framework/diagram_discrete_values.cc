#include "drake/systems/framework/diagram_discrete_values.h"

namespace drake {
namespace systems {

template <typename T>
DiagramDiscreteValues<T>::~DiagramDiscreteValues() {}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::DiagramDiscreteValues);
