#include "drake/multibody/fem/acceleration_newmark_scheme.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

template <typename T>
std::unique_ptr<DiscreteTimeIntegrator<T>>
AccelerationNewmarkScheme<T>::DoClone() const {
  return std::make_unique<AccelerationNewmarkScheme<T>>(dt(), gamma_, beta_);
}

template <typename T>
void AccelerationNewmarkScheme<T>::DoUpdateStateFromChangeInUnknowns(
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
void AccelerationNewmarkScheme<T>::DoAdvanceOneTimeStep(
    const FemState<T>& prev_state, const VectorX<T>& z,
    FemState<T>* state) const {
  const VectorX<T>& an = prev_state.GetAccelerations();
  const VectorX<T>& vn = prev_state.GetVelocities();
  const VectorX<T>& qn = prev_state.GetPositions();
  const VectorX<T>& a = z;
  /* Note that the partials of the next time step's (q, v, a) w.r.t. z are
   (δt²*β, δt*γ, 1), and they must match the weights given by DoGetWeights().
  */
  state->SetPositions(qn + dt() * vn +
                      dt() * dt() * (beta_ * a + (0.5 - beta_) * an));
  state->SetVelocities(vn + dt() * (gamma_ * a + (1.0 - gamma_) * an));
  state->SetAccelerations(a);
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::fem::internal::AccelerationNewmarkScheme);
