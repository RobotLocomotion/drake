#include "drake/multibody/fem/discrete_time_integrator.h"

// TODO(xuchenhan-tri): Add unit tests for this class.

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
Vector3<T> DiscreteTimeIntegrator<T>::weights() const {
  return do_get_weights();
}

template <typename T>
const VectorX<T>& DiscreteTimeIntegrator<T>::GetUnknowns(
    const FemState<T>& state) const {
  return DoGetUnknowns(state);
}

template <typename T>
void DiscreteTimeIntegrator<T>::UpdateStateFromChangeInUnknowns(
    const VectorX<T>& dz, FemState<T>* state) const {
  DRAKE_DEMAND(state != nullptr);
  DRAKE_DEMAND(dz.size() == state->num_dofs());
  DoUpdateStateFromChangeInUnknowns(dz, state);
}

template <typename T>
void DiscreteTimeIntegrator<T>::AdvanceOneTimeStep(
    const FemState<T>& prev_state, const VectorX<T>& unknown_variable,
    FemState<T>* next_state) const {
  DRAKE_DEMAND(next_state != nullptr);
  DRAKE_DEMAND(prev_state.num_dofs() == next_state->num_dofs());
  DRAKE_DEMAND(prev_state.num_dofs() == unknown_variable.size());
  DoAdvanceOneTimeStep(prev_state, unknown_variable, next_state);
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::DiscreteTimeIntegrator)
