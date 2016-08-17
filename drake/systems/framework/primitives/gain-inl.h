#pragma once

/// @file
/// Template method implementations for gain.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/systems/framework/primitives/gain.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

template <typename T>
Gain<T>::Gain(const T& k, int length) : gain_(k) {
  // TODO(amcastro-tri): remove the length parameter from the constructor once
  // #3109 supporting automatic lengths is resolved.
  this->DeclareInputPort(kVectorValued, length, kContinuousSampling);
  this->DeclareOutputPort(kVectorValued, length, kContinuousSampling);
}

template <typename T>
void Gain<T>::EvalOutput(const ContextBase<T>& context,
                          SystemOutput<T>* output) const {
  // Checks that the single output port has the correct length.
  // Checks on the output structure are assertions, not exceptions,
  // since failures would reflect a bug in the Gain implementation, not
  // user error setting up the system graph. They do not require unit test
  // coverage, and should not run in release builds.

  DRAKE_ASSERT(System<T>::IsValidOutput(*output));
  DRAKE_ASSERT(System<T>::IsValidContext(context));

  // There is only one input.
  // TODO(amcastro-tri): Solve #3140 so that the next line reads:
  // auto& input_vector = System<T>::get_input_vector(context, 0);
  // where the return is an Eigen expression.
  const VectorInterface<T>* input_vector = context.get_vector_input(0);

  VectorInterface<T>* output_vector =
      output->get_mutable_port(0)->GetMutableVectorData();

  // TODO(amcastro-tri): Solve #3140 so that we can readily access the Eigen
  // vector like so:
  // auto& output_vector = System<T>::get_output_vector(context, 0);
  // where the return is an Eigen expression.
  output_vector->get_mutable_value() = gain_ * input_vector->get_value();
}

}  // namespace systems
}  // namespace drake
