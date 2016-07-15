#include "drake/systems/framework/primitives/adder3.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

template <typename T>
Adder3<T>::Adder3(const std::string& name, int num_inputs, int length)
    : System3<T>(name), num_inputs_(num_inputs), length_(length) {
  for (int i = 0; i < num_inputs; ++i)
    AddInputPort(std::make_unique<VectorInputPort3<T>>(length));

  AddOutputPort(std::make_unique<VectorOutputPort3<T>>(length));

  // No state variables. No parameters. No interest in time.
}

template <typename T>
void Adder3<T>::DoCalcOutputPort(const AbstractContext3& abstract_context,
                                 int port_num, AbstractValue* result) const {
  // Check that the single output port has the correct length, then zero it.
  // Checks on the output structure are assertions, not exceptions,
  // since failures would reflect a bug in the Adder implementation, not
  // user error setting up the system graph. They do not require unit test
  // coverage, and should not run in release builds.
  DRAKE_ASSERT(port_num == 0);
  VectorInterface<T>* output_vector = to_mutable_vector_interface<T>(result);
  DRAKE_ASSERT(output_vector != nullptr);
  DRAKE_ASSERT(output_vector->get_value().rows() == length_);
  output_vector->get_mutable_value() = VectorX<T>::Zero(length_);

  const Context3<T>& context =
      dynamic_cast<const Context3<T>&>(abstract_context);

  // Check that there are the expected number of input ports.
  if (context.get_num_input_ports() != num_inputs_) {
    throw std::out_of_range("Adder3<T>::DoCalcOutputPort(): Expected " +
                            std::to_string(num_inputs_) +
                            "input ports in Context, but had " +
                            std::to_string(context.get_num_input_ports()));
  }

  // Sum each input port into the output.
  for (int i = 0; i < context.get_num_input_ports(); i++)
    output_vector->get_mutable_value() +=
        EvalVectorInputPort(context, i).get_value();
}

template class Adder3<double>;

}  // namespace systems
}  // namespace drake
