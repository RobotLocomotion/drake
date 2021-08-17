#pragma once

#include "drake/common/default_scalars.h"
#include "drake/multibody/fixed_fem/dev/newmark_scheme.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* Implements the interface StateUpdater with Newmark-beta time integration
 scheme with velocity being the unknown variable. Given the value for the
 current time step velocity `v`, the states are calculated from states from the
 previous time step according to the following equations:

      a = (v - vₙ) / (dt ⋅ γ) - (1 − γ) / γ ⋅ aₙ
      x = xₙ + dt ⋅ (β/γ ⋅ v +  (1 - β/γ) ⋅ vₙ) + dt² ⋅ (0.5 − β/γ) ⋅ aₙ.

 See NewmarkScheme for the reference to the Newmark-beta integration scheme.
 See AccelerationNewmarkScheme for the same integration scheme implemented with
 acceleration as the unknown variable.
 @tparam_nonsymbolic_scalar */
template <typename T>
class VelocityNewmarkScheme final : public NewmarkScheme<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VelocityNewmarkScheme);

  /* Constructs a Newmark scheme with velocity as the unknown variable.
   @pre dt_in > 0.
   @pre 0.5 <= gamma_in <= 1.
   @pre 0 <= beta_in <= 0.5. */
  VelocityNewmarkScheme(double dt_in, double gamma_in, double beta_in)
      : NewmarkScheme<T>(dt_in, gamma_in, beta_in),
        beta_over_gamma_(beta_in / gamma_in),
        one_over_dt_gamma_(1.0 / (dt() * gamma())) {}

  ~VelocityNewmarkScheme() = default;

 private:
  using NewmarkScheme<T>::dt;
  using NewmarkScheme<T>::gamma;

  Vector3<T> do_get_weights() const final {
    return {beta_over_gamma_ * dt(), 1.0, one_over_dt_gamma_};
  }

  const VectorX<T>& DoGetUnknowns(const FemStateBase<T>& state) const final {
    return state.qdot();
  }

  void DoUpdateStateFromChangeInUnknowns(const VectorX<T>& dz,
                                         FemStateBase<T>* state) const final;

  void DoAdvanceOneTimeStep(const FemStateBase<T>& prev_state,
                            const VectorX<T>& unknown_variable,
                            FemStateBase<T>* state) const final;

  double beta_over_gamma_;
  double one_over_dt_gamma_;
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::VelocityNewmarkScheme)
