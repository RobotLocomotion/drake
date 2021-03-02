#pragma once

#include "drake/common/unused.h"
#include "drake/multibody/fixed_fem/dev/fem_state.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
// TODO(xuchenhan-tri): Template this class on scalar type only so that
//  FemModelBase can own it.
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
  Vector3<T> weights() const { return do_get_weights(); }

  /** Updates the FemState `state` given the change in the unknown variable
   `dz`.
   @pre state != nullptr.
   @pre dz.size() == state.num_generalized_positions(). */
  void UpdateStateFromChangeInUnknowns(const VectorX<T>& dz,
                                       State* state) const {
    DRAKE_DEMAND(state != nullptr);
    DRAKE_DEMAND(dz.size() == state->num_generalized_positions());
    DoUpdateStateFromChangeInUnknowns(dz, state);
  }

  // TODO(xuchenhan-tri): Consider passing in more than one previous state when
  //  we implement multi-step methods. */
  /** Advance the given `state` by one time step.
   More specifically, if State::ode_order() > 0, then advance `state` in time
   using the discrete time stepping scheme by a time step of size `dt` which is
   stored in the time stepping scheme.
   If State::ode_order() == 0, throw an exception.
   @param[in] prev_state The state at the previous time step.
   @param[in, out] state The state at the current time step. The highest order
   state will be used as input (and will not be modified by this method). For
   example, if State::ode_order() == 2, then `state->qddot()` will be used as
   input (and will not be modified) and `state->q()` and `state->qdot()` will be
   modified by numerically integrating in time.
   @pre state != nullptr. */
  void AdvanceOneTimeStep(const State& prev_state,
                          const VectorX<T>& highest_order_state,
                          State* state) const {
    DRAKE_DEMAND(state != nullptr);
    DoAdvanceOneTimeStep(prev_state, highest_order_state, state);
  }

 protected:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StateUpdater);
  StateUpdater() = default;

  /** Derived classes must override this method to provide an implementation to
   the method weights(). */
  virtual Vector3<T> do_get_weights() const = 0;

  /** Derived classes must override this method to udpate the FemState `state`
   given the change in the unknown variable `dz` based on the time-stepping
   scheme of the derived class. The `dz` provided here has compatible size with
   the number of generalized positions in `state` and does not need to be
   checked again.  */
  virtual void DoUpdateStateFromChangeInUnknowns(const VectorX<T>& dz,
                                                 State* state) const = 0;

  /** Derived classes templated on FemState whose ode_order() is greater than 0
   must override this method to advance a single time step according to the
   specific time stepping scheme. */
  virtual void DoAdvanceOneTimeStep(const State& prev_state,
                                    const VectorX<T>& highest_order_state,
                                    State* state) const {
    unused(prev_state, highest_order_state, state);
    if constexpr (State::ode_order() == 0) {
      throw std::logic_error(
          "There is no notion of time in a zeroth order ODE.");
    }
    throw std::logic_error(
        "AdvanceOneTimeStep() should be implemented for this StateUpdater but "
        "is "
        "not implemented.");
  }
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
