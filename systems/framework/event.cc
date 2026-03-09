#include "drake/systems/framework/event.h"

namespace drake {
namespace systems {

template <typename T>
Event<T>::~Event() = default;

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Event);
