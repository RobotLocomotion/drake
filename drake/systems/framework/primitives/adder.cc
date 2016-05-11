#include "drake/systems/framework/primitives/adder.h"

#include <cassert>
#include <stdexcept>
#include <string>

#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

template <typename T>
Adder<T>::Adder(int num_inputs, int length)
    : num_inputs_(num_inputs), length_(length) {}

template <typename T>
Context<T> Adder<T>::CreateDefaultContext() const {
  Context<T> context;
  context.continuous_inputs.resize(num_inputs_);
  return context;
}

template <typename T>
SystemOutput<T> Adder<T>::CreateDefaultOutput() const {
  // An adder has just one output port, a BasicVector of the size specified
  // at construction time.
  SystemOutput<T> output;
  {
    ContinuousOutputPort<T> port;
    port.output.reset(new BasicVector<T>(length_));
    output.continuous_ports.push_back(std::move(port));
  }
  return output;
}

template <typename T>
void Adder<T>::Output(const Context<T>& context, Cache<T>* cache,
                      SystemOutput<T>* output) const {
  // Check that the single output port has the correct length, then zero it.
  // Checks on the output structure are assertions, not exceptions,
  // since failures would reflect a bug in the Adder implementation, not
  // user error setting up the system graph. They do not require unit test
  // coverage, and should not run in release builds.
  assert(output->continuous_ports.size() == 1);
  VectorInterface<T>* output_port = output->continuous_ports[0].output.get();
  assert(output_port != nullptr);
  assert(output_port->get_value().rows() == length_);
  for (int i = 0; i < output_port->get_value().rows(); i++) {
    output_port->get_mutable_value()[i] = 0;
  }

  // Check that there are the expected number of input ports.
  if (context.continuous_inputs.size() != num_inputs_) {
    throw std::runtime_error("Expected " + std::to_string(num_inputs_) +
                             "input ports, but had " +
                             std::to_string(context.continuous_inputs.size()));
  }

  // Sum each input port into the output, after checking that it has the
  // expected length.
  for (int i = 0; i < context.continuous_inputs.size(); i++) {
    const VectorInterface<T>* input = context.continuous_inputs[i];
    if (input == nullptr || input->get_value().rows() != length_) {
      throw std::runtime_error("Input port " + std::to_string(i) +
                               "is nullptr or has incorrect size.");
    }
    for (int j = 0; j < input->get_value().rows(); j++) {
      output_port->get_mutable_value()[j] += input->get_value()[j];
    }
  }
}

template <typename T>
void Adder<T>::GetDerivativesOfGeneralizedPosition(
    const Context<T>& context, Cache<T>* cache,
    VectorInterface<T>* derivatives) const {
  derivatives->set_value(VectorX<T>::Zero(0));
}

template <typename T>
void Adder<T>::GetDerivativesOfGeneralizedVelocity(
    const Context<T>& context, Cache<T>* cache,
    VectorInterface<T>* derivatives) const {
  derivatives->set_value(VectorX<T>::Zero(0));
}

template <typename T>
void Adder<T>::GetDerivativesOfOtherContinuousState(
    const Context<T>& context, Cache<T>* cache,
    VectorInterface<T>* derivatives) const {
  derivatives->set_value(VectorX<T>::Zero(0));
}

template <typename T>
void Adder<T>::MapVelocityToConfigurationDerivative(
    const Context<T>& context, Cache<T>* cache,
    VectorInterface<T>* derivatives) const {
  derivatives->set_value(VectorX<T>::Zero(0));
}

template class DRAKESYSTEMFRAMEWORK_EXPORT Adder<double>;

}  // namespace systems
}  // namesapce drake
