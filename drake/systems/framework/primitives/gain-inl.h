#pragma once

/// @file
/// Template method implementations for gain.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/systems/framework/primitives/gain.h"

#include <sstream>
#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {

// TODO(amcastro-tri): remove the size parameter from the constructor once
// #3109 supporting automatic sizes is resolved.
template <typename T>
Gain<T>::Gain(const T& k, int size) : Gain(VectorX<T>::Ones(size) * k) { }

template <typename T>
Gain<T>::Gain(const VectorX<T>& k) : k_(k) {
  DRAKE_DEMAND(k.size() > 0);
  this->DeclareInputPort(kVectorValued, k.size(), kContinuousSampling);
  this->DeclareOutputPort(kVectorValued, k.size(), kContinuousSampling);
}

template <typename T>
const T& Gain<T>::get_gain() const {
  if (!k_.isConstant(k_[0])) {
    std::stringstream s;
    s << "The gain vector, [" << k_ << "], cannot be represented as a scalar "
      << "value. Please use drake::systems::Gain::get_gain_vector() instead.";
    DRAKE_ABORT_MSG(s.str().c_str());
  }
  return k_[0];
}

template <typename T>
const VectorX<T>& Gain<T>::get_gain_vector() const {
  return k_;
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
      k_.array() * input_vector.array();
}

}  // namespace systems
}  // namespace drake
