#include "drake/systems/framework/parameters.h"

namespace drake {
namespace systems {

template <typename T>
Parameters<T>::~Parameters() {}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Parameters);
