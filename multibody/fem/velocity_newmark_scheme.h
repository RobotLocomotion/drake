#pragma once

#include "drake/common/default_scalars.h"
#include "drake/multibody/fem/discrete_time_integrator.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Implements the interface DiscreteTimeIntegrator with Newmark-beta time
 integration scheme. Given the value for the next time step velocity `vₙ₊₁`,
 the states are calculated from states from the previous time step according to
 the following equations:

      aₙ₊₁ = (vₙ₊₁ - vₙ) / (δt ⋅ γ) - (1 − γ) / γ ⋅ aₙ
      qₙ₊₁ = qₙ + δt ⋅ (β/γ ⋅ vₙ₊₁ +  (1 - β/γ) ⋅ vₙ) + δt² ⋅ (0.5 − β/γ) ⋅ aₙ.

 Note that the scheme is unconditionally unstable for gamma < 0.5 and therefore
 we require gamma >= 0.5.
 See AccelerationNewmarkScheme for the same integration scheme implemented with
 acceleration as the unknown variable.
 See [Newmark, 1959] for the original reference for the method.

 [Newmark, 1959] Newmark, Nathan M. "A method of computation for structural
 dynamics." Journal of the engineering mechanics division 85.3 (1959): 67-94.
 @tparam_nonsymbolic_scalar */
template <typename T>
class VelocityNewmarkScheme final : public DiscreteTimeIntegrator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VelocityNewmarkScheme);

  /* Constructs a Newmark scheme with velocity as the unknown variable.
   @pre dt_in > 0.
   @pre 0.5 <= gamma <= 1.
   @pre 0 <= beta <= 0.5. */
  VelocityNewmarkScheme(double dt_in, double gamma, double beta)
      : DiscreteTimeIntegrator<T>(dt_in),
        gamma_(gamma),
        beta_over_gamma_(beta / gamma),
        one_over_dt_gamma_(1.0 / (dt_in * gamma)) {
    DRAKE_DEMAND(0.5 <= gamma && gamma <= 1);
    DRAKE_DEMAND(0 <= beta && beta <= 0.5);
  }

  ~VelocityNewmarkScheme() = default;

 private:
  using DiscreteTimeIntegrator<T>::dt;

  Vector3<T> DoGetWeights() const final {
    return {beta_over_gamma_ * dt(), 1.0, one_over_dt_gamma_};
  }

  const VectorX<T>& DoGetUnknowns(const FemState<T>& state) const final {
    return state.GetVelocities();
  }

  void DoUpdateStateFromChangeInUnknowns(const VectorX<T>& dz,
                                         FemState<T>* state) const final;

  void DoAdvanceOneTimeStep(const FemState<T>& prev_state, const VectorX<T>& z,
                            FemState<T>* state) const final;

  double gamma_{};
  double beta_over_gamma_{};
  double one_over_dt_gamma_{};
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::VelocityNewmarkScheme)
