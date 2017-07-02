#pragma once

/// @file
/// Template method implementations for zero_order_hold.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/common/unused.h"
#include "drake/systems/primitives/zero_order_hold.h"

namespace drake {
namespace systems {

template <typename T>
ZeroOrderHold<T>::ZeroOrderHold(double period_sec, int size)
    : VectorSystem<T>(size, size), period_sec_(period_sec) {
  // TODO(david-german-tri): remove the size parameter from the constructor
  // once #3109 supporting automatic sizes is resolved.
  this->DeclareDiscreteState(size);
  this->DeclarePeriodicDiscreteUpdate(period_sec);
}

template <typename T>
void ZeroOrderHold<T>::DoCalcVectorOutput(
      const Context<T>&,
      const Eigen::VectorBlock<const VectorX<T>>& input,
      const Eigen::VectorBlock<const VectorX<T>>& state,
      Eigen::VectorBlock<VectorX<T>>* output) const {
  unused(input);
  *output = state;
}

template <typename T>
void ZeroOrderHold<T>::DoCalcVectorDiscreteVariableUpdates(
    const Context<T>&,
    const Eigen::VectorBlock<const VectorX<T>>& input,
    const Eigen::VectorBlock<const VectorX<T>>& state,
    Eigen::VectorBlock<VectorX<T>>* discrete_updates) const {
  unused(state);
  *discrete_updates = input;
}

template <typename T>
ZeroOrderHold<symbolic::Expression>* ZeroOrderHold<T>::DoToSymbolic() const {
  return new ZeroOrderHold<symbolic::Expression>(period_sec_,
                                                 this->get_input_port().size());
}

}  // namespace systems
}  // namespace drake
