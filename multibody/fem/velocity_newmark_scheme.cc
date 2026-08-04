#include "drake/multibody/fem/velocity_newmark_scheme.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
std::unique_ptr<DiscreteTimeIntegrator<T>> VelocityNewmarkScheme<T>::DoClone()
    const {
  return std::make_unique<VelocityNewmarkScheme<T>>(dt(), gamma_,
                                                    gamma_ * beta_over_gamma_);
}

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
  /* Note that the partials of the next time step's (q, v, a) w.r.t. z are
   (β*δt/γ, 1, 1/(δt*γ)), and they must match the weights given by
   DoGetWeights(). */
  state->SetPositions(
      qn + dt() * (beta_over_gamma_ * v + (1.0 - beta_over_gamma_) * vn) +
      dt() * dt() * (0.5 - beta_over_gamma_) * an);
  state->SetAccelerations(one_over_dt_gamma_ * (v - vn) -
                          (1.0 - gamma_) / gamma_ * an);
  state->SetVelocities(v);
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::fem::internal::VelocityNewmarkScheme);
