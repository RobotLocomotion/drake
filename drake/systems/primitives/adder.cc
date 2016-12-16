#include "drake/systems/primitives/adder.h"

#include <stdexcept>
#include <string>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {

template <typename T>
Adder<T>::Adder(int num_inputs, int size) {
  for (int i = 0; i < num_inputs; i++) {
    this->DeclareInputPort(kVectorValued, size);
  }
  this->DeclareOutputPort(kVectorValued, size);
}

template <typename T>
const SystemPortDescriptor<T>& Adder<T>::get_output_port() const {
  return System<T>::get_output_port(0);
}

template <typename T>
void Adder<T>::DoCalcOutput(const Context<T>& context,
                            SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  BasicVector<T>* output_vector = output->GetMutableVectorData(0);

  // Zeroes the output.
  const int n = static_cast<int>(output_vector->get_value().rows());
  output_vector->get_mutable_value() = VectorX<T>::Zero(n);

  // Sum each input port into the output, after checking that it has the
  // expected size.
  for (int i = 0; i < context.get_num_input_ports(); i++) {
    const BasicVector<T>* input_vector = this->EvalVectorInput(context, i);
    output_vector->get_mutable_value() += input_vector->get_value();
  }
}

template <typename T>
Adder<AutoDiffXd>* Adder<T>::DoToAutoDiffXd() const {
  return new Adder<AutoDiffXd>(this->get_num_input_ports(),
                               this->get_input_port(0).get_size());
}

// Explicitly instantiates on the most common scalar types.
template class Adder<double>;
template class Adder<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
