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

  output_port_ = &this->DeclareVectorOutputPort(
      BasicVector<T>(size), &Adder<T>::CalcSum);
}

template <typename T>
void Adder<T>::CalcSum(const Context<T>& context,
                       BasicVector<T>* sum) const {
  Eigen::VectorBlock<VectorX<T>> sum_vector = sum->get_mutable_value();

  // Zeroes the output.
  sum_vector.setZero();

  // Sum each input port into the output.
  for (int i = 0; i < context.get_num_input_ports(); i++) {
    const BasicVector<T>* input_vector = this->EvalVectorInput(context, i);
    sum_vector += input_vector->get_value();
  }
}

template <typename T>
Adder<AutoDiffXd>* Adder<T>::DoToAutoDiffXd() const {
  return new Adder<AutoDiffXd>(this->get_num_input_ports(),
                               this->get_input_port(0).size());
}

template <typename T>
Adder<symbolic::Expression>* Adder<T>::DoToSymbolic() const {
  return new Adder<symbolic::Expression>(this->get_num_input_ports(),
                                         this->get_input_port(0).size());
}

// Explicitly instantiates on the most common scalar types.
template class Adder<double>;
template class Adder<AutoDiffXd>;
template class Adder<symbolic::Expression>;

}  // namespace systems
}  // namespace drake
