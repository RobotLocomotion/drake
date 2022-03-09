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
  const VectorX<T>& q = state->GetPositions();
  const Vector3<T> weights = this->GetWeights();
  state->SetPositions(q + weights(0) * dz);
  state->SetVelocities(v + weights(1) * dz);
  state->SetAccelerations(a + weights(2) * dz);
}

template <typename T>
void VelocityNewmarkScheme<T>::DoAdvanceOneTimeStep(
    const FemState<T>& prev_state, const VectorX<T>& z,
    FemState<T>* state) const {
  const VectorX<T>& an = prev_state.GetAccelerations();
  const VectorX<T>& vn = prev_state.GetVelocities();
  const VectorX<T>& qn = prev_state.GetPositions();
  const VectorX<T>& v = z;
  const Vector3<T> weights = this->GetWeights();
  state->SetPositions(qn + weights(0) * v +
                      dt() * ((1.0 - beta_over_gamma_) * vn +
                              dt() * (0.5 - beta_over_gamma_) * an));
  state->SetAccelerations(weights(2) * v - one_over_dt_gamma_ * vn -
                          (1.0 - gamma_) / gamma_ * an);
  state->SetVelocities(weights(1) * v);
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::VelocityNewmarkScheme)
