#pragma once

#include "drake/common/default_scalars.h"
#include "drake/multibody/fixed_fem/dev/newmark_scheme.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Implements NewmarkScheme with acceleration as the unknown variable.
 Given the value for the current time step acceleration `a`, the current state
 can be calculated from the state from the previous time step according to the
 following equations:

      v = vₙ + dt ⋅ (γ ⋅ a + (1−γ) ⋅ aₙ)
      x = xₙ + dt ⋅ vₙ + dt² ⋅ [β ⋅ a + (0.5−β) ⋅ aₙ].

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

  Vector3<T> do_get_weights() const final {
    return {beta() * dt() * dt(), gamma() * dt(), 1.0};
  }

  const VectorX<T>& DoGetUnknowns(const FemStateBase<T>& state) const final {
    return state.qddot();
  }

  void DoUpdateStateFromChangeInUnknowns(const VectorX<T>& dz,
                                         FemStateBase<T>* state) const final;

  void DoAdvanceOneTimeStep(const FemStateBase<T>& prev_state,
                            const VectorX<T>& unknown_variable,
                            FemStateBase<T>* state) const final;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::AccelerationNewmarkScheme)
