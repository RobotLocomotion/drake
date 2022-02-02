#include "drake/multibody/fem/acceleration_newmark_scheme.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
void AccelerationNewmarkScheme<T>::DoUpdateStateFromChangeInUnknowns(
    const VectorX<T>& dz, FemState<T>* state) const {
  const VectorX<T>& a = state->GetAccelerations();
  const VectorX<T>& v = state->GetVelocities();
  const VectorX<T>& x = state->GetPositions();
  state->SetAccelerations(a + dz);
  state->SetVelocities(v + dt() * gamma() * dz);
  state->SetPositions(x + dt() * dt() * beta() * dz);
}

template <typename T>
void AccelerationNewmarkScheme<T>::DoAdvanceOneTimeStep(
    const FemState<T>& prev_state, const VectorX<T>& unknown_variable,
    FemState<T>* state) const {
  const VectorX<T>& an = prev_state.GetAccelerations();
  const VectorX<T>& vn = prev_state.GetVelocities();
  const VectorX<T>& xn = prev_state.GetPositions();
  const VectorX<T>& a = unknown_variable;
  state->SetAccelerations(a);
  state->SetVelocities(vn + dt() * (gamma() * a + (1.0 - gamma()) * an));
  state->SetPositions(xn + dt() * vn +
              dt() * dt() * (beta() * a + (0.5 - beta()) * an));
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::AccelerationNewmarkScheme)
