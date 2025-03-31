#include "drake/systems/analysis/explicit_euler_integrator.h"

namespace drake {
namespace systems {

template <class T>
ExplicitEulerIntegrator<T>::~ExplicitEulerIntegrator() = default;

template <class T>
std::unique_ptr<IntegratorBase<T>> ExplicitEulerIntegrator<T>::DoClone() const {
  return std::make_unique<ExplicitEulerIntegrator>(
      this->get_system(), this->get_maximum_step_size());
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::ExplicitEulerIntegrator);
