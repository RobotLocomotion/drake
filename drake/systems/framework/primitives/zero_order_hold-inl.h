#pragma once

/// @file
/// Template method implementations for zero_order_hold.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/systems/framework/primitives/zero_order_hold.h"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/difference_state.h"
#include "drake/systems/framework/leaf_context.h"
#include "drake/systems/framework/system_port_descriptor.h"

namespace drake {
namespace systems {

template <typename T>
ZeroOrderHold<T>::ZeroOrderHold(const T& period_sec, int size) {
  // TODO(david-german-tri): remove the size parameter from the constructor
  // once #3109 supporting automatic sizes is resolved.
  this->DeclareInputPort(kVectorValued, size, kContinuousSampling);
  this->DeclareOutputPort(kVectorValued, size, kDiscreteSampling);
  this->DeclareUpdatePeriodSec(period_sec);
}

template <typename T>
void ZeroOrderHold<T>::EvalOutput(const Context<T>& context,
                                  SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  System<T>::GetMutableOutputVector(output, 0) =
      context.get_difference_state(0)->CopyToVector();
}

template <typename T>
void ZeroOrderHold<T>::DoEvalDifferenceUpdates(
    const Context<T>& context, DifferenceState<T>* difference_state) const {
  DRAKE_DEMAND(difference_state->size() == 1);
  difference_state->get_mutable_difference_state(0)->SetFromVector(
      this->EvalVectorInput(context, 0)->get_value());
}

template <typename T>
std::unique_ptr<DifferenceState<T>> ZeroOrderHold<T>::AllocateDifferenceState()
    const {
  // The zero-order hold's state is first-order. Its state vector size is the
  // same as the input (and output) vector size.
  const int size = System<T>::get_output_port(0).get_size();
  DRAKE_DEMAND(System<T>::get_input_port(0).get_size() == size);
  std::vector<std::unique_ptr<BasicVector<T>>> xd;
  xd.push_back(std::make_unique<BasicVector<T>>(size));
  return std::make_unique<DifferenceState<T>>(std::move(xd));
}

}  // namespace systems
}  // namespace drake
