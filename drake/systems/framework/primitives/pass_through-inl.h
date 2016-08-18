#pragma once

/// @file
/// Template method implementations for pass_through.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/systems/framework/primitives/pass_through.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

template <typename T>
PassThrough<T>::PassThrough(int length) {
  // TODO(amcastro-tri): remove the length parameter from the constructor once
  // #3109 supporting automatic lengths is resolved.
  this->DeclareInputPort(kVectorValued, length, kInheritedSampling);
  this->DeclareOutputPort(kVectorValued, length, kInheritedSampling);
}

template <typename T>
void PassThrough<T>::EvalOutput(const ContextBase<T>& context,
                          SystemOutput<T>* output) const {
  DRAKE_ASSERT(System<T>::IsValidOutput(*output));
  DRAKE_ASSERT(System<T>::IsValidContext(context));

  VectorInterface<T>* output_vector =
      output->get_mutable_port(0)->GetMutableVectorData();

  // TODO(amcastro-tri): Solve #3140 so that the next line reads:
  // auto& input_vector = System<T>::get_input_vector(context, 0);
  // where the return is an Eigen expression.
  const VectorInterface<T>* input_vector = context.get_vector_input(0);

  // TODO(amcastro-tri): Solve #3140 so that we can readily access the Eigen
  // vector like so:
  // auto& output_vector = System<T>::get_output_vector(context, 0);
  // where the return is an Eigen expression.
  // TODO(amcastro-tri): the output should simply reference the input port's
  // value to avoid copy.
  output_vector->get_mutable_value() = input_vector->get_value();
}

}  // namespace systems
}  // namespace drake
