#pragma once

#include "drake/multibody/fixed_fem/dev/state_updater.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
/** Implements the interface StateUpdater with the zeroth-order state updater.
 Namely, q = z and dq = dz.
 @tparam State    The type of FemState to be updated by this %StateUpdater. The
 template parameter State must be an instantiation of FemState.
 @pre State::ode_order() == 0. */
template <class State>
class ZerothOrderStateUpdater final : public StateUpdater<State> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ZerothOrderStateUpdater);

  static_assert(State::ode_order() == 0);
  using T = typename State::T;

  ZerothOrderStateUpdater() = default;
  ~ZerothOrderStateUpdater() = default;

 private:
  /* Implements StateUpdater::weights(). */
  Vector3<T> do_get_weights() const final { return {1, 0, 0}; }

  /* Implements StateUpdater::DoUpdateStateFromChangeInUnknowns(). */
  void DoUpdateStateFromChangeInUnknowns(const VectorX<T>& dz,
                                                  State* state) const final {
    state->SetQ(state->q() + dz);
  }
};
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
