#pragma once

/// @file
/// Template method implementations for zero_order_hold.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/systems/framework/primitives/zero_order_hold.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {

template <typename T>
ZeroOrderHold<T>::ZeroOrderHold(const T& period_sec, int length) {
  // TODO(david-german-tri): remove the length parameter from the constructor
  // once #3109 supporting automatic lengths is resolved.
  this->DeclareInputPort(kVectorValued, length, kInheritedSampling);
  this->DeclareOutputPort(kVectorValued, length, DiscreteSampling(period_sec));
  this->DeclarePeriodSec(period_sec);
}

template <typename T>
void ZeroOrderHold<T>::EvalOutput(const Context<T>& context,
                                  SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  System<T>::GetMutableOutputVector(output, 0) =
      context.get_difference_state()->CopyToVector();
}

template <typename T>
void ZeroOrderHold<T>::DoUpdate(const Context<T>& context,
                                VectorBase<T>* difference_state) const {
  difference_state->SetFromVector(
      this->EvalVectorInput(context, 0)->get_value());
}

template <typename T>
std::unique_ptr<VectorBase<T>>
ZeroOrderHold<T>::AllocateDifferenceState() const {
  // The zero-order hold's state is first-order. Its state vector length is the
  // same as the input (and output) vector length.
  const int length = System<T>::get_output_port(0).get_size();
  DRAKE_ASSERT(System<T>::get_input_port(0).get_size() == length);
  return std::make_unique<BasicVector<T>>(length);
}

}  // namespace systems
}  // namespace drake
