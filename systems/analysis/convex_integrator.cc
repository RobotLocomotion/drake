#include "drake/systems/analysis/convex_integrator.h"

namespace drake {
namespace systems {

template <class T>
bool ConvexIntegrator<T>::DoStep(const T& h) {
  Context<T>& context = *this->get_mutable_context();
  VectorBase<T>& x = context.get_mutable_continuous_state_vector();

  // TODO(vincekurtz): update the state
  // For now just some placeholder nonesense so we can see some dynamics
  const T x0 = x.GetAtIndex(0);
  x.SetAtIndex(0, x0 + h);

  return true;  // step was successful
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::ConvexIntegrator);
