#include "drake/systems/primitives/shared_pointer_system.h"

namespace drake {
namespace systems {

template <typename T>
SharedPointerSystem<T>::SharedPointerSystem(std::shared_ptr<void> held,
                                            std::type_index held_type)
    : LeafSystem<T>(SystemTypeTag<SharedPointerSystem>{}),
      held_(std::move(held)),
      held_type_(held_type) {}

template <typename T>
template <typename U>
SharedPointerSystem<T>::SharedPointerSystem(const SharedPointerSystem<U>& other)
    : SharedPointerSystem<T>(other.held_, other.held_type_) {}

template <typename T>
SharedPointerSystem<T>::~SharedPointerSystem() = default;

template <typename T>
typename LeafSystem<T>::GraphvizFragment
SharedPointerSystem<T>::DoGetGraphvizFragment(
    const typename LeafSystem<T>::GraphvizFragmentParams&) const {
  // Do not show anything in Graphviz for this system.
  return {};
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::SharedPointerSystem);
