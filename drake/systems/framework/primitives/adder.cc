#include "drake/systems/framework/primitives/adder.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

template <typename T>
Adder<T>::Adder(int num_inputs, int length)
    : num_inputs_(num_inputs), length_(length) {}

template <typename T>
std::unique_ptr<Context<T>> Adder<T>::CreateDefaultContext() const {
  std::unique_ptr<Context<T>> context(new Context<T>);
  context->get_mutable_input()->ports.resize(num_inputs_);
  return context;
}

template <typename T>
std::unique_ptr<SystemOutput<T>> Adder<T>::AllocateOutput() const {
  // An adder has just one output port, a BasicVector of the size specified
  // at construction time.
  std::unique_ptr<SystemOutput<T>> output(new SystemOutput<T>);
  {
    OutputPort<T> port;
    port.vector_output.reset(new BasicVector<T>(length_));
    output->ports.push_back(std::move(port));
  }
  return output;
}

template <typename T>
void Adder<T>::Output(const Context<T>& context,
                      SystemOutput<T>* output) const {
  // Check that the single output port has the correct length, then zero it.
  // Checks on the output structure are assertions, not exceptions,
  // since failures would reflect a bug in the Adder implementation, not
  // user error setting up the system graph. They do not require unit test
  // coverage, and should not run in release builds.
  DRAKE_ASSERT(output->ports.size() == 1);
  VectorInterface<T>* output_port = output->ports[0].vector_output.get();
  DRAKE_ASSERT(output_port != nullptr);
  DRAKE_ASSERT(output_port->get_value().rows() == length_);
  output_port->get_mutable_value() = VectorX<T>::Zero(length_);

  // Check that there are the expected number of input ports.
  if (context.get_input().ports.size() != num_inputs_) {
    throw std::out_of_range(
        "Expected " + std::to_string(num_inputs_) + "input ports, but had " +
        std::to_string(context.get_input().ports.size()));
  }

  // Sum each input port into the output, after checking that it has the
  // expected length.
  for (int i = 0; i < context.get_input().ports.size(); i++) {
    const VectorInterface<T>* input =
        context.get_input().ports[i].vector_input;
    if (input == nullptr || input->get_value().rows() != length_) {
      throw std::out_of_range("Input port " + std::to_string(i) +
                              "is nullptr or has incorrect size.");
    }
    output_port->get_mutable_value() += input->get_value();
  }
}

template class DRAKESYSTEMFRAMEWORK_EXPORT Adder<double>;

}  // namespace systems
}  // namespace drake
