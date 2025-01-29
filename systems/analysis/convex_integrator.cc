#include "drake/systems/analysis/convex_integrator.h"

namespace drake {
namespace systems {

template <class T>
bool ConvexIntegrator<T>::DoStep(const T& h) {
  Context<T>& context = *this->get_mutable_context();
  VectorBase<T>& x = context.get_mutable_continuous_state_vector();

  // TODO(vincekurtz): update the state
  (void)h;
  (void)x;

  return true;  // step was successful
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::systems::ConvexIntegrator);
