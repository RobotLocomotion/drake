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
  const Vector3<T> weights = this->GetWeights();
  state->SetPositions(qn + dt() * vn + weights(0) * a +
                      dt() * dt() * (0.5 - beta_) * an);
  state->SetVelocities(vn + weights(1) * a + dt() * (1.0 - gamma_) * an);
  state->SetAccelerations(weights(2) * a);
}

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::AccelerationNewmarkScheme)
