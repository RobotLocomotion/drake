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

  // TODO(amcastro-tri): the output should simply reference the input port's
  // value to avoid copy.
  System<T>::get_mutable_output_vector(*output, 0) =
      System<T>::get_input_vector(context, 0);
}

}  // namespace systems
}  // namespace drake
