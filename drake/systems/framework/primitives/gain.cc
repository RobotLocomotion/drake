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
Gain<T>::Gain(double k, int length) : gain_(k), length_(length) {
  //TODO(amcastro-tri):
  // parameter length should be used to specify the system's input port and
  // does not need to be stored in member length_.
  // The constructor for a simple system like this should read:
  //add_input_port(make_unique<InputPort>("in", length));
  //add_output_port(make_unique<OutputPort>("out", length));
}

template <typename T>
std::unique_ptr<ContextBase<T>> Gain<T>::CreateDefaultContext() const {
  std::unique_ptr<Context<T>> context(new Context<T>);
  // TODO(amcastro-tri): System<T> should provide a default implementation of
  // this method since state, inputs and outputs were specified in the
  // constructor.

  // A Gain block only has one input port.
  context->SetNumInputPorts(1);
  return std::unique_ptr<ContextBase<T>>(context.release());
}

template <typename T>
std::unique_ptr<SystemOutput<T>> Gain<T>::AllocateOutput(
    const ContextBase<T>& context) const {
  // TODO(amcastro-tri): System<T> should provide a default implementation.
  // Notice that his code exactly matches the one in adder.cc and therefore can
  // be reused.

  // A Gain has just one output port, a BasicVector of the size specified
  // at construction time.
  std::unique_ptr<LeafSystemOutput<T>> output(new LeafSystemOutput<T>);
  {
    std::unique_ptr<BasicVector<T>> data(new BasicVector<T>(length_));
    std::unique_ptr<OutputPort<T>> port(new OutputPort<T>(std::move(data)));
    output->get_mutable_ports()->push_back(std::move(port));
  }
  return std::unique_ptr<SystemOutput<T>>(output.release());
}

template <typename T>
void Gain<T>::EvalOutput(const ContextBase<T>& context,
                          SystemOutput<T>* output) const {
  // Check that the single output port has the correct length, then zero it.
  // Checks on the output structure are assertions, not exceptions,
  // since failures would reflect a bug in the Gain implementation, not
  // user error setting up the system graph. They do not require unit test
  // coverage, and should not run in release builds.

  // TODO(amcastro-tri): These asserts are not needed here since they should
  // be performed by Diagram::Finalize().
  DRAKE_ASSERT(output->get_num_ports() == 1);
  VectorInterface<T>* output_vector =
      output->get_mutable_port(0)->GetMutableVectorData();
  DRAKE_ASSERT(output_vector != nullptr);
  DRAKE_ASSERT(output_vector->get_value().rows() == length_);

  // Check that there are the expected number of input ports.
  if (context.get_num_input_ports() != 1) {
    throw std::out_of_range("Expected only one input port, but had " +
        std::to_string(context.get_num_input_ports()));
  }

  // There is only one input.
  // TODO(amcastro-tri): This line should read:
  // input_vector = System<T>::get_input_port(0).
  const VectorInterface<T>* input = context.get_vector_input(0);

  // Check the expected length.
  // TODO(amcastro-tri): Another test that is not needed here.
  if (input == nullptr || input->get_value().rows() != length_) {
    throw std::out_of_range("Input port is nullptr or has incorrect size.");
  }

  // The actual one liner that needs to be written.
  // TODO(amcastro-tri): System<T> should provide interfaces to directly get the
  // actually useful Eigen vectors like so:
  // auto input_vector = System<T>::get_input_port(0).get_vector(context);
  // auto output_vector =
  //   System<T>::get_output_port(0).get_mutable_vector(context);
  output_vector->get_mutable_value() = gain_ * input->get_value();
}

template class DRAKESYSTEMFRAMEWORK_EXPORT Gain<double>;

}  // namespace systems
}  // namespace drake
