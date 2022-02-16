#pragma once

#include "drake/common/default_scalars.h"
#include "drake/multibody/fem/newmark_scheme.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Implements NewmarkScheme with acceleration as the unknown variable.
 Given the value for the current time step acceleration `a`, the state at the
 next time step can be calculated from that of the previous time step according
 to the following equations:

      v = vₙ + dt ⋅ (γ ⋅ a + (1−γ) ⋅ aₙ)
      q = qₙ + dt ⋅ vₙ + dt² ⋅ [β ⋅ a + (0.5−β) ⋅ aₙ].

 See NewmarkScheme for the reference to the Newmark-beta integration scheme.
 See VelocityNewmarkScheme for the same integration scheme implemented with
 velocity as the unknown variable.
 @tparam_nonsymbolic_scalar */
template <typename T>
class AccelerationNewmarkScheme final : public NewmarkScheme<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AccelerationNewmarkScheme);

  /* Constructs a Newmark scheme with acceleration as the unknown variable.
   @pre dt_in > 0.
   @pre 0.5 <= gamma_in <= 1.
   @pre 0 <= beta_in <= 0.5. */
  AccelerationNewmarkScheme(double dt_in, double gamma_in, double beta_in)
      : NewmarkScheme<T>(dt_in, gamma_in, beta_in) {}

  ~AccelerationNewmarkScheme() = default;

 private:
  using NewmarkScheme<T>::dt;
  using NewmarkScheme<T>::gamma;
  using NewmarkScheme<T>::beta;

  Vector3<T> DoGetWeights() const final {
    return {beta() * dt() * dt(), gamma() * dt(), 1.0};
  }

  const VectorX<T>& DoGetUnknowns(const FemState<T>& state) const final {
    return state.GetAccelerations();
  }

  void DoUpdateStateFromChangeInUnknowns(const VectorX<T>& dz,
                                         FemState<T>* state) const final;

  void DoAdvanceOneTimeStep(const FemState<T>& prev_state,
                            const VectorX<T>& z,
                            FemState<T>* state) const final;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::AccelerationNewmarkScheme)
