#pragma once

#include "drake/multibody/fixed_fem/dev/state_updater.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** Implements the interface StateUpdater with Newmark-beta time integration
 scheme. Given the value for the current time step accleration `a`, the states
 are calculated from states from the previous time step according to the
 following equations:

      v = vₙ + dt ⋅ (γ ⋅ a + (1−γ) ⋅ aₙ)
      x = xₙ + dt ⋅ vₙ + dt² ⋅ [β ⋅ a + (0.5−β) ⋅ aₙ].

 See [Newmark, 1959] for the original reference for the method.

 [Newmark, 1959] Newmark, Nathan M. "A method of computation for structural
 dynamics." Journal of the engineering mechanics division 85.3 (1959): 67-94.

 @tparam State    The type of FemState to be updated by the %NewmarkScheme. The
 template parameter State must be an instantiation of FemState.
 @pre State::ode_order() == 2. */
template <class State>
class NewmarkScheme final : public StateUpdater<State> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NewmarkScheme);

  static_assert(State::ode_order() == 2);
  using T = typename State::T;

  /** Construct a Newmark scheme with the provided timestep, `gamma` and `beta`.
   @pre dt > 0.
   @pre 0 <= gamma <= 1.
   @pre 0 <= beta <= 0.5. */
  NewmarkScheme(double dt, double gamma, double beta)
      : dt_(dt), gamma_(gamma), beta_(beta) {
    DRAKE_DEMAND(dt > 0);
    DRAKE_DEMAND(0 <= gamma && gamma <= 1);
    DRAKE_DEMAND(0 <= beta && beta <= 0.5);
  }

  ~NewmarkScheme() = default;

 private:
  /* Implements StateUpdater::weights(). */
  Vector3<T> do_get_weights() const final {
    return {beta_ * dt_ * dt_, gamma_ * dt_, 1.0};
  }

  /* Implements StateUpdater::DoUpdateState(). */
  void DoUpdateState(const VectorX<T>& dz, State* state) const final {
    const VectorX<T>& a = state->qddot();
    const VectorX<T>& v = state->qdot();
    const VectorX<T>& x = state->q();
    state->set_qddot(a + dz);
    state->set_qdot(v + dt_ * gamma_ * dz);
    state->set_q(x + dt_ * dt_ * beta_ * dz);
  }

  /* Implements StateUpdater::DoAdvanceOneTimeStep(). */
  void DoAdvanceOneTimeStep(const State& prev_state, State* state) const final {
    const VectorX<T>& an = prev_state.qddot();
    const VectorX<T>& vn = prev_state.qdot();
    const VectorX<T>& xn = prev_state.q();
    const VectorX<T>& a = state->qddot();
    state->set_qdot(vn + dt_ * (gamma_ * a + (1.0 - gamma_) * an));
    state->set_q(xn + dt_ * vn + dt_ * dt_ * (beta_ * a + (0.5 - beta_) * an));
  }

  double dt_{0};
  double gamma_{0.5};
  double beta_{0.25};
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
