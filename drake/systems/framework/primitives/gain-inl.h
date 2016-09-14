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
#include "drake/systems/framework/leaf_context.h"

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
const T& Gain<T>::get_gain() const {
  return gain_;
}

template <typename T>
void Gain<T>::EvalOutput(const Context<T>& context,
                         SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  auto input_vector = this->EvalEigenVectorInput(context, 0);
  System<T>::GetMutableOutputVector(output, 0) = gain_ * input_vector;
}

}  // namespace systems
}  // namespace drake
