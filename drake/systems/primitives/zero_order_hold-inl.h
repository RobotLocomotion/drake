#pragma once

/// @file
/// Template method implementations for zero_order_hold.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace systems {

template <typename T>
ZeroOrderHold<T>::ZeroOrderHold(const T& period_sec, int size)
    : SisoVectorSystem<T>(size, size) {
  // TODO(david-german-tri): remove the size parameter from the constructor
  // once #3109 supporting automatic sizes is resolved.
  this->DeclareDiscreteState(size);
  this->DeclareDiscreteUpdatePeriodSec(period_sec);
}

template <typename T>
void ZeroOrderHold<T>::DoCalcVectorOutput(
      const Context<T>& context,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const {
  *output = state;
}

template <typename T>
void ZeroOrderHold<T>::DoCalcVectorDiscreteVariableUpdates(
    const Context<T>& context,
    const Eigen::VectorBlock<const VectorX<T>>& input,
    const Eigen::VectorBlock<const VectorX<T>>& state,
    Eigen::VectorBlock<VectorX<T>>* discrete_updates) const {
  *discrete_updates = input;
}

}  // namespace systems
}  // namespace drake
