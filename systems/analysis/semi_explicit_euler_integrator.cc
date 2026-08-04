#include "drake/systems/analysis/semi_explicit_euler_integrator.h"

namespace drake {
namespace systems {

template <class T>
SemiExplicitEulerIntegrator<T>::~SemiExplicitEulerIntegrator() = default;

template <class T>
std::unique_ptr<IntegratorBase<T>> SemiExplicitEulerIntegrator<T>::DoClone()
    const {
  return std::make_unique<SemiExplicitEulerIntegrator>(
      this->get_system(), this->get_maximum_step_size());
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::SemiExplicitEulerIntegrator);
