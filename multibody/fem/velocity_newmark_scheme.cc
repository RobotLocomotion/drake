#include "drake/multibody/fem/velocity_newmark_scheme.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
void VelocityNewmarkScheme<T>::DoUpdateStateFromChangeInUnknowns(
    const VectorX<T>& dz, FemState<T>* state) const {
  const VectorX<T>& a = state->GetAccelerations();
  const VectorX<T>& v = state->GetVelocities();
  const VectorX<T>& x = state->GetPositions();
  state->SetAccelerations(a + one_over_dt_gamma_ * dz);
  state->SetVelocities(v + dz);
  state->SetPositions(x + dt() * beta_over_gamma_ * dz);
}

template <typename T>
void VelocityNewmarkScheme<T>::DoAdvanceOneTimeStep(
    const FemState<T>& prev_state, const VectorX<T>& unknown_variable,
    FemState<T>* state) const {
  const VectorX<T>& an = prev_state.GetAccelerations();
  const VectorX<T>& vn = prev_state.GetVelocities();
  const VectorX<T>& xn = prev_state.GetPositions();
  const VectorX<T>& v = unknown_variable;
  state->SetPositions(
      xn + dt() * (beta_over_gamma_ * v + (1.0 - beta_over_gamma_) * vn) +
      dt() * dt() * (0.5 - beta_over_gamma_) * an);
  state->SetAccelerations(one_over_dt_gamma_ * (v - vn) -
                          (1.0 - gamma()) / gamma() * an);
  state->SetVelocities(v);
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::VelocityNewmarkScheme)
