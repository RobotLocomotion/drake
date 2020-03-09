#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {

template <typename T>
LeafSystem<T>::~LeafSystem() {}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::LeafSystem)
