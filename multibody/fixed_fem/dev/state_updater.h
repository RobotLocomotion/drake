#pragma once

#include "drake/multibody/fixed_fem/dev/fem_state.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** %StateUpdater provides the interface to update FemState in NewtonSolver.
 In each Newton-Raphson iteration of the FEM solver, we are solving for an
 equation in the form of
        G(q, q̇, q̈) = 0.
 For different FemModels, G takes different forms. For dynamic elasticity,
        G(q, q̇, q̈) = Mq̈ - fₑ(q) - fᵥ(q, q̇) - fₑₓₜ,
 where M is the mass matrix, fₑ(q) is the elastic force, fᵥ(q, q̇) is the
 damping force, and fₑₓₜ are the external forces. For static elasticity,
        G(q, q̇, q̈) = G(q) = -fₑ(q) + fₑₓₜ.
 For the Poisson equation,
        G(q, q̇, q̈) = G(q) = Kq − f.
 With time discretization, one can express `q`, `q̇,` `q̈` in terms of a
 single variable, which we dub the name "key variable" and denote it with `z`.
 For example, for dynamic elasticity with Newmark scheme,
        q̈ = z;
        q̇ = q̇ₙ + dt ⋅ (γ ⋅ z + (1−γ) ⋅ q̈ₙ)
        q = qₙ + dt ⋅ q̇ₙ + dt² ⋅ [β ⋅ z + (0.5−β) ⋅ q̈ₙ].
 Hence, we can write the system of equations as
        G(z) = 0,
 and in each Newton-Raphson iteration, we solve for a linear system of equation
 of the form
        ∇G(z) ⋅ dz = -G(z),
 where ∇G = ∂G/∂z. StateUpdater is responsible for updating the FemState given
 the solution of the linear solve, `dz`. In addition, %StateUpdater also
 provides the derivatives of the states with respect to the key variable `z`,
 namely, `∂q/∂z`,  `∂q̇/∂z`, and `∂q̈/∂z`, that are needed for building ∇G.
 @tparam State    The type of FemState to be updated by this %StateUpdater. The
 template parameter State must be an instantiation of FemState. */
template <class State>
class StateUpdater {
 public:
  using T = typename State::T;

  virtual ~StateUpdater() = default;

  /** For a representative degree of freedom i, returns the derivative of the
   state with respect to the key variable `zᵢ`, [∂qᵢ/∂zᵢ, ∂q̇ᵢ/∂zᵢ, ∂q̈ᵢ/∂zᵢ].
   The choice of i is arbitrary as these derivatives are the same for all
   degrees of freedom in the same model. These derivatives can be used as
   weights to combine stiffness, damping and mass matrices to form the tangent
   matrix. If q̇ or q̈ are not a part of the state, their derivatives are set to
   0. */
  virtual Vector3<T> weights() const = 0;

  /** Updates the FemState `state` given the change in the value of the key
   variable `z`.
   @pre state != nullptr.
   @pre dz.size() == state.num_generalized_positions(). */
  void UpdateState(const VectorX<T>& dz, State* state) const {
    DRAKE_DEMAND(state != nullptr);
    DRAKE_DEMAND(dz.size() == state->num_generalized_positions());
    DoUpdateState(dz, state);
  }

 protected:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StateUpdater);
  StateUpdater() = default;

  /** Derived classes must override this method to udpate the FemState `state`
   given the key variable `dz` based on the time-stepping scheme of the derived
   class. The `dz` provided here has compatible size with the number of
   generalized positions in `state` and does not need to be checked again.  */
  virtual void DoUpdateState(const VectorX<T>& dz, State* state) const = 0;
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
