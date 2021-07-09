#pragma once

#include "drake/multibody/fixed_fem/dev/state_updater.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
/* Implements the interface StateUpdater with the zeroth-order state updater.
 Namely, q = z and dq = dz.
 @tparam_nonsymbolic_scalar */
template <typename T>
class ZerothOrderStateUpdater final : public StateUpdater<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ZerothOrderStateUpdater);

  ZerothOrderStateUpdater() = default;
  ~ZerothOrderStateUpdater() = default;

 private:
  Vector3<T> do_get_weights() const final { return {1, 0, 0}; }

  const VectorX<T>& DoGetUnknowns(const FemStateBase<T>& state) const final {
    return state.q();
  }

  void DoUpdateStateFromChangeInUnknowns(const VectorX<T>& dz,
                                         FemStateBase<T>* state) const final {
    state->SetQ(state->q() + dz);
  }
};
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
