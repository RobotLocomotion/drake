#include "drake/systems/framework/wrapped_system.h"

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {
namespace internal {

// N.B. The shared_ptr constructor is defined in wrapped_system_builder.cc.

template <typename T>
WrappedSystem<T>::~WrappedSystem() = default;

template <typename T>
const System<T>& WrappedSystem<T>::unwrap() const {
  auto diagram_subsystems = this->GetSystems();
  return *diagram_subsystems.at(0);
}

}  // namespace internal
}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::internal::WrappedSystem);
