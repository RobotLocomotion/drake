#pragma once

#include <memory>

#include "drake/common/default_scalars.h"
#include "drake/multibody/fem/discrete_time_integrator.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Implements the interface DiscreteTimeIntegrator with Newmark-beta time
 integration scheme. Given the value for the next time step acceleration `aₙ₊₁`,
 the states are calculated from states from the previous time step according to
 the following equations:

      vₙ₊₁ = vₙ + δt ⋅ (γ ⋅ aₙ₊₁ + (1 − γ) ⋅ aₙ)
      qₙ₊₁ = qₙ + δt ⋅ vₙ + δt² ⋅ [β ⋅ aₙ₊₁ + (0.5 − β) ⋅ aₙ].

 Note that the scheme is unconditionally unstable for gamma < 0.5 and therefore
 we require gamma >= 0.5.
 See VelocityNewmarkScheme for the same integration scheme implemented with
 velocity as the unknown variable.
 See [Newmark, 1959] for the original reference for the method.

 [Newmark, 1959] Newmark, Nathan M. "A method of computation for structural
 dynamics." Journal of the engineering mechanics division 85.3 (1959): 67-94.
 @tparam_default_scalar */
template <typename T>
class AccelerationNewmarkScheme final : public DiscreteTimeIntegrator<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AccelerationNewmarkScheme);

  /* Constructs a Newmark scheme with acceleration as the unknown variable.
   @pre dt_in > 0.
   @pre 0.5 <= gamma <= 1.
   @pre 0 <= beta <= 0.5. */
  AccelerationNewmarkScheme(double dt_in, double gamma, double beta)
      : DiscreteTimeIntegrator<T>(dt_in), gamma_(gamma), beta_(beta) {
    DRAKE_DEMAND(0.5 <= gamma && gamma <= 1);
    DRAKE_DEMAND(0 <= beta && beta <= 0.5);
  }

  ~AccelerationNewmarkScheme() = default;

  using DiscreteTimeIntegrator<T>::dt;

 private:
  std::unique_ptr<DiscreteTimeIntegrator<T>> DoClone() const final;

  /* The weights much match the partials in DoAdvanceOneTimeStep(). */
  Vector3<T> DoGetWeights() const final {
    return {beta_ * dt() * dt(), gamma_ * dt(), 1.0};
  }

  const VectorX<T>& DoGetUnknowns(const FemState<T>& state) const final {
    return state.GetAccelerations();
  }

  void DoUpdateStateFromChangeInUnknowns(const VectorX<T>& dz,
                                         FemState<T>* state) const final;

  void DoAdvanceOneTimeStep(const FemState<T>& prev_state, const VectorX<T>& z,
                            FemState<T>* state) const final;

  const double gamma_{};
  const double beta_{};
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::fem::internal::AccelerationNewmarkScheme);
