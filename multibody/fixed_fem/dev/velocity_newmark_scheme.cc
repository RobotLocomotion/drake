#include "drake/multibody/fixed_fem/dev/velocity_newmark_scheme.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
void VelocityNewmarkScheme<T>::DoUpdateStateFromChangeInUnknowns(
    const VectorX<T>& dz, FemStateBase<T>* state) const {
  const VectorX<T>& a = state->qddot();
  const VectorX<T>& v = state->qdot();
  const VectorX<T>& x = state->q();
  state->SetQddot(a + one_over_dt_gamma_ * dz);
  state->SetQdot(v + dz);
  state->SetQ(x + dt() * beta_over_gamma_ * dz);
}

template <typename T>
void VelocityNewmarkScheme<T>::DoAdvanceOneTimeStep(
    const FemStateBase<T>& prev_state, const VectorX<T>& unknown_variable,
    FemStateBase<T>* state) const {
  const VectorX<T>& an = prev_state.qddot();
  const VectorX<T>& vn = prev_state.qdot();
  const VectorX<T>& xn = prev_state.q();
  const VectorX<T>& v = unknown_variable;
  state->SetQddot(one_over_dt_gamma_ * (v - vn) -
                  (1.0 - gamma()) / gamma() * an);
  state->SetQdot(v);
  state->SetQ(xn +
              dt() * (beta_over_gamma_ * v + (1.0 - beta_over_gamma_) * vn) +
              dt() * dt() * (0.5 - beta_over_gamma_) * an);
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::VelocityNewmarkScheme)
