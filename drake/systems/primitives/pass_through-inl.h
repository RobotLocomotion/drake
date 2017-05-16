#pragma once

/// @file
/// Template method implementations for pass_through.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/common/unused.h"
#include "drake/systems/primitives/pass_through.h"

namespace drake {
namespace systems {

// TODO(amcastro-tri): remove the size parameter from the constructor once
// #3109 supporting automatic sizes is resolved.
template <typename T>
PassThrough<T>::PassThrough(int size) : SisoVectorSystem<T>(size, size) { }

template <typename T>
void PassThrough<T>::DoCalcVectorOutput(
    const Context<T>&,
    const Eigen::VectorBlock<const VectorX<T>>& input,
    const Eigen::VectorBlock<const VectorX<T>>& state,
    Eigen::VectorBlock<VectorX<T>>* output) const {
  unused(state);
  *output = input;
}

template <typename T>
PassThrough<symbolic::Expression>* PassThrough<T>::DoToSymbolic() const {
  return new PassThrough<symbolic::Expression>(this->get_input_port().size());
}

}  // namespace systems
}  // namespace drake
