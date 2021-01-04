#pragma once

#include "drake/multibody/fixed_fem/dev/state_updater.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace internal {
/* Implements the interface StateUpdater with Newmark-beta time integration
 scheme. The states are updated according to the following equation:
            v = vₙ + dt ⋅ (γ ⋅ aₙ + (1−γ) ⋅ a)
      x = xₙ + dt ⋅ vₙ + dt² ⋅ [β ⋅ a + (0.5−β) ⋅ aₙ].
 See [Newmark, 1959] for the original reference for the method.
 
 [Newmark, 1959] Newmark, Nathan M. "A method of computation for structural
 dynamics." Journal of the engineering mechanics division 85.3 (1959): 67-94.
 
 @tparam State    The type of FemState to be updated by this %StateUpdater. The
 template parameter State must be an instantiation of FemState.
 @pre State::ode_order() == 2. */
template <class State>
class NewmarkScheme final : public StateUpdater<State> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NewmarkScheme);

  static_assert(State::ode_order() == 2);
  using T = State::T;

  /* Construct a Newmark scheme with the provided timestep, `gamma` and `beta`.
   */
  NewmarkScheme(double dt, double gamma, double beta)
      : dt_(dt), gamma_(gamma), beta_(beta) {}

  ~NewmarkScheme() = default;

  /** Implements StateUpdater::state_derivatives(). */
  Vector3<T> state_derivatives() final const {
    return {beta_ * dt_ * dt_, (1.0 - gamma_) * dt_, 1.0};
  }

 private:
  /* Implements StateUpdater::DoUpdateState(). */
  void DoUpdateState(const VectorX<T>& dz, State* state) final const {
    double dt_sqr = dt_ * dt_;
    const VectorX<T>& an = state->qddot();
    const VectorX<T>& vn = state->qdot();
    state->set_q(state()->q() + dt * vn + dt_sqrt * (an + beta_ * dz));
    state->set_qdot(vn + dt * (an + (1.0 - gamma_) * dz));
    state->set_qddot(an + dz);
  }

  double dt_{0};
  double gamma_{0.5};
  double beta_{0.25};
};
}  // namespace internal
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
