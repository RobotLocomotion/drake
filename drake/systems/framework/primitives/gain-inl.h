#pragma once

/// @file
/// Template method implementations for gain.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/systems/framework/primitives/gain.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {

template <typename T>
Gain<T>::Gain(const T& k, int size) : Gain(VectorX<T>::Ones(size) * k) { }

template <typename T>
Gain<T>::Gain(const VectorX<T>& gain) : gain_(gain) {
  // TODO(amcastro-tri): remove the size parameter from the constructor once
  // #3109 supporting automatic sizes is resolved.
  this->DeclareInputPort(kVectorValued, gain.size(), kContinuousSampling);
  this->DeclareOutputPort(kVectorValued, gain.size(), kContinuousSampling);
}

template <typename T>
const VectorX<T>& Gain<T>::get_gain() const {
  return gain_;
}

template <typename T>
const SystemPortDescriptor<T>& Gain<T>::get_input_port() const {
  return System<T>::get_input_port(0);
}

template <typename T>
const SystemPortDescriptor<T>& Gain<T>::get_output_port() const {
  return System<T>::get_output_port(0);
}

template <typename T>
void Gain<T>::EvalOutput(const Context<T>& context,
                         SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  auto input_vector = this->EvalEigenVectorInput(context, 0);
  System<T>::GetMutableOutputVector(output, 0) =
      gain_.array() * input_vector.array();
}

}  // namespace systems
}  // namespace drake
