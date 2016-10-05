#pragma once

/// @file
/// Template method implementations for pass_through.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/systems/framework/primitives/pass_through.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {

template <typename T>
PassThrough<T>::PassThrough(int size) {
  // TODO(amcastro-tri): remove the size parameter from the constructor once
  // #3109 supporting automatic sizes is resolved.
  this->DeclareInputPort(kVectorValued, size, kInheritedSampling);
  // TODO(david-german-tri): Provide a way to infer the type.
  this->DeclareOutputPort(kVectorValued, size, kInheritedSampling);
}

template <typename T>
void PassThrough<T>::EvalOutput(const Context<T>& context,
                                SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  // TODO(amcastro-tri): the output should simply reference the input port's
  // value to avoid copy.
  System<T>::GetMutableOutputVector(output, 0) =
      System<T>::EvalEigenVectorInput(context, 0);
}

}  // namespace systems
}  // namespace drake
