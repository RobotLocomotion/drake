#include "drake/systems/framework/primitives/adder.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/drakeSystemFramework_export.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

template <typename T>
Adder<T>::Adder(int num_inputs, int length) {
  for (int i = 0; i < num_inputs; i++) {
    this->DeclareInputPort(kVectorValued, length, kInheritedSampling);
  }
  this->DeclareOutputPort(kVectorValued, length, kInheritedSampling);
}

template <typename T>
void Adder<T>::EvalOutput(const ContextBase<T>& context,
                          SystemOutput<T>* output) const {
  // Checks on the output structure are assertions, not exceptions,
  // since failures would reflect a bug in the Adder implementation, not
  // user error setting up the system graph. They do not require unit test
  // coverage, and should not run in release builds.

  DRAKE_ASSERT(System<T>::IsValidOutput(*output));
  DRAKE_ASSERT(System<T>::IsValidContext(context));

  VectorInterface<T>* output_vector =
      output->get_mutable_port(0)->GetMutableVectorData();

  // Zeroes the output.
  const int n = output_vector->get_value().rows();
  output_vector->get_mutable_value() = VectorX<T>::Zero(n);

  // Sum each input port into the output, after checking that it has the
  // expected length.
  for (int i = 0; i < context.get_num_input_ports(); i++) {
    const VectorInterface<T>* input_vector = context.get_vector_input(i);
    output_vector->get_mutable_value() += input_vector->get_value();
  }
}

// Explicitly instantiates on the most common scalar types.
template class DRAKESYSTEMFRAMEWORK_EXPORT Adder<double>;

}  // namespace systems
}  // namespace drake
