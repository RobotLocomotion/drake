#include "drake/multibody/fixed_fem/dev/acceleration_newmark_scheme.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
void AccelerationNewmarkScheme<T>::DoUpdateStateFromChangeInUnknowns(
    const VectorX<T>& dz, FemStateBase<T>* state) const {
  const VectorX<T>& a = state->qddot();
  const VectorX<T>& v = state->qdot();
  const VectorX<T>& x = state->q();
  state->SetQddot(a + dz);
  state->SetQdot(v + dt() * gamma() * dz);
  state->SetQ(x + dt() * dt() * beta() * dz);
}

template <typename T>
void AccelerationNewmarkScheme<T>::DoAdvanceOneTimeStep(
    const FemStateBase<T>& prev_state, const VectorX<T>& unknown_variable,
    FemStateBase<T>* state) const {
  const VectorX<T>& an = prev_state.qddot();
  const VectorX<T>& vn = prev_state.qdot();
  const VectorX<T>& xn = prev_state.q();
  const VectorX<T>& a = unknown_variable;
  state->SetQddot(a);
  state->SetQdot(vn + dt() * (gamma() * a + (1.0 - gamma()) * an));
  state->SetQ(xn + dt() * vn +
              dt() * dt() * (beta() * a + (0.5 - beta()) * an));
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::AccelerationNewmarkScheme)
