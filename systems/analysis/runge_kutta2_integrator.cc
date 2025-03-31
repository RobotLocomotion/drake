#include "drake/systems/analysis/runge_kutta2_integrator.h"

namespace drake {
namespace systems {

template <class T>
RungeKutta2Integrator<T>::~RungeKutta2Integrator() = default;

template <class T>
std::unique_ptr<IntegratorBase<T>> RungeKutta2Integrator<T>::DoClone() const {
  return std::make_unique<RungeKutta2Integrator>(this->get_system(),
                                                 this->get_maximum_step_size());
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::RungeKutta2Integrator);
