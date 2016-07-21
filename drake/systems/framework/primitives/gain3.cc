#include "drake/systems/framework/primitives/gain3.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

template <typename T>
Gain3<T>::Gain3(const std::string& name, const T& gain, int length)
    : System3<T>(name), gain_(gain), length_(length) {
  this->AddInputPort(std::make_unique<VectorInputPort3<T>>(length));
  this->AddOutputPort(std::make_unique<VectorOutputPort3<T>>(length));

  // No state variables. No parameters. No interest in time.
}

template <typename T>
void Gain3<T>::DoCalcOutputPort(const AbstractContext3& abstract_context,
                                int port_num, AbstractValue* result) const {
  // Check that the single output port has the correct length, then zero it.
  // Checks on the output structure are assertions, not exceptions,
  // since failures would reflect a bug in the Gain implementation, not
  // user error setting up the system graph. They do not require unit test
  // coverage, and should not run in release builds.
  DRAKE_ASSERT(port_num == 0);
  VectorInterface<T>* output_vector = to_mutable_vector_interface<T>(result);
  DRAKE_ASSERT(output_vector != nullptr);
  DRAKE_ASSERT(output_vector->get_value().rows() == length_);

  const Context3<T>& context =
      dynamic_cast<const Context3<T>&>(abstract_context);

  // Check that there are the expected number of input ports.
  if (context.get_num_input_ports() != 1) {
    throw std::out_of_range(
        "Gain3<T>::DoCalcOutputPort(): Expected one input "
        "port in Context, but had " +
        std::to_string(context.get_num_input_ports()));
  }

  output_vector->get_mutable_value() =
      gain_ * this->EvalVectorInputPort(context, 0).get_value();
}

template class Gain3<double>;

}  // namespace systems
}  // namespace drake
