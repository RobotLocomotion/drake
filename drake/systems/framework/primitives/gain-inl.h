#pragma once

/// @file
/// Template method implementations for see gain.h.
/// Most users should only include that file, not this one.
/// For background, @see http://drake.mit.edu/cxx_inl.html.

#include "drake/systems/framework/primitives/gain.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"

#include <iostream>
#define PRINT_VAR(x) std::cout <<  #x ": " << x << std::endl;

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

  DRAKE_ASSERT(output->get_num_ports() == 1);
  VectorInterface<T>* output_vector =
      output->get_mutable_port(0)->GetMutableVectorData();
  DRAKE_ASSERT(output_vector != nullptr);
  DRAKE_ASSERT(output_vector->get_value().rows() ==
      static_cast<int>(this->get_output_ports().size()));

  // Check that there are the expected number of input ports.
  if (context.get_num_input_ports() != 1) {
    throw std::out_of_range("Expected only one input port, but had " +
        std::to_string(context.get_num_input_ports()));
  }

  // There is only one input.
  // TODO(amcastro-tri): This line should read:
  // InputPort& input_port = System<T>::get_input_port(0);
  // auto& input_vector = input_port.get_vector(context); // where the return is
  // an Eigen expression.
  // A plausible alternative would be:
  // auto& input_vector = System<T>::get_input_vector(context, 0);
  const VectorInterface<T>* input_vector = context.get_vector_input(0);

  // Check the expected length.
  DRAKE_ASSERT(input_vector != nullptr);
  DRAKE_ASSERT(input_vector->get_value().rows() ==
      this->get_input_port(0).get_size());

  // TODO(amcastro-tri): System<T> should provide interfaces to directly get the
  // actually useful Eigen vectors like so:
  // auto input_vector = System<T>::get_input_port(0).get_vector(context);
  // auto output_vector =
  //   System<T>::get_output_port(0).get_mutable_vector(context);
  output_vector->get_mutable_value() = gain_ * input_vector->get_value();
}

}  // namespace systems
}  // namespace drake
