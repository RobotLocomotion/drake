#pragma once

#include "drake/common/default_scalars.h"
#include "drake/multibody/fem/fem_state.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {

/* DiscreteTimeIntegrator is an abstract class that encapsulates discrete time
 integrations schemes for second order ODEs. When a second order ODE

     f(q, v = q̇, a = q̈) = 0

 is discretized in time, the quantities of interest evaluated at the next time
 step can often be expressed as an affine mapping on a single variable z, i.e.

        qₙ₊₁ = αₚ z + bₚ
        vₙ₊₁ = αᵥ z + bᵥ
        aₙ₊₁ = αₐ z + bₐ

 For example, for the Newmark-beta scheme, if one chooses z = a, we get

        qₙ₊₁ = qₙ + dt ⋅ vₙ + dt² ⋅ [β ⋅ z + (0.5−β) ⋅ aₙ].
        vₙ₊₁ = vₙ + dt ⋅ (γ ⋅ z + (1−γ) ⋅ aₙ)
        aₙ₊₁ = z;

 On the other hand, if one chooses z = v instead for the same scheme, we get

        qₙ₊₁ = xₙ + dt ⋅ (β/γ ⋅ z +  (1 - β/γ) ⋅ vₙ) + dt² ⋅ (0.5 − β/γ) ⋅ aₙ.
        vₙ₊₁ = z
        aₙ₊₁ = (z - vₙ) / (dt ⋅ γ) - (1 − γ) / γ ⋅ aₙ

 DiscreteTimeIntegrator provides the interface to query the relationship between
 the states (`q`, `v`, and `a`) and the unknown variable `z`.
 @tparam_nonsymbolic_scalar */
template <typename T>
class DiscreteTimeIntegrator {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DiscreteTimeIntegrator);

  virtual ~DiscreteTimeIntegrator() = default;

  /* Returns (αₚ, αᵥ, αₐ), the derivative of (q, v, a) with respect to the
   unknown variable z (See class documentation). These weights can be used to
   combine stiffness, damping, and mass matrices to form the tangent
   matrix (see FemModel::CalcTangentMatrix). */
  Vector3<T> weights() const;

  /* Extracts the unknown variable `z` from the given FEM `state`. */
  const VectorX<T>& GetUnknowns(const FemState<T>& state) const;

  /* Updates the FemState `state` given the change in the unknown variables.
   More specifically, it sets the given `state` to the following values.

        q = αₚ (z + dz) + bₚ
        v = αᵥ (z + dz) + bᵥ
        a = αₐ (z + dz) + bₐ

   @pre state != nullptr.
   @pre dz.size() == state->num_dofs(). */
  void UpdateStateFromChangeInUnknowns(const VectorX<T>& dz,
                                       FemState<T>* state) const;

  /* Advances `prev_state` by one time step to the `next_state` with the given
   value of the unknown variable z.
   @param[in]  prev_state        The state at the previous time step.
   @param[in]  unknown_variable  The unknown variable z.
   @param[out] next_state        The state at the next time step.
   @pre next_state != nullptr.
   @pre The sizes of `prev_state`, `unknown_variable`, and `next_state` are
   compatible. */
  void AdvanceOneTimeStep(const FemState<T>& prev_state,
                          const VectorX<T>& unknown_variable,
                          FemState<T>* next_state) const;

  /* Returns the discrete time step of the integration scheme. */
  double dt() const { return dt_; }

 protected:
  explicit DiscreteTimeIntegrator(double dt) : dt_(dt) {
    DRAKE_THROW_UNLESS(dt > 0);
  }

  /* Derived classes must override this method to implement the NVI
   get_weights(). */
  virtual Vector3<T> do_get_weights() const = 0;

  /* Derived classes must override this method to implement the NVI
   GetUnknowns(). */
  virtual const VectorX<T>& DoGetUnknowns(const FemState<T>& state) const = 0;

  /* Derived classes must override this method to implement the NVI
   UpdateStateFromChangeInUnknowns(). */
  virtual void DoUpdateStateFromChangeInUnknowns(const VectorX<T>& dz,
                                                 FemState<T>* state) const = 0;

  /* Derived classes must override this method to implement the NVI
   AdvanceOneTimeStep(). */
  virtual void DoAdvanceOneTimeStep(const FemState<T>& prev_state,
                                    const VectorX<T>& unknowns,
                                    FemState<T>* next_state) const = 0;

  double dt_{0.0};
};

}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::multibody::fem::internal::DiscreteTimeIntegrator)
