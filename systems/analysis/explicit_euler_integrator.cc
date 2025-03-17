#include "drake/systems/analysis/explicit_euler_integrator.h"

namespace drake {
namespace systems {

template <class T>
ExplicitEulerIntegrator<T>::~ExplicitEulerIntegrator() = default;

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::ExplicitEulerIntegrator);
