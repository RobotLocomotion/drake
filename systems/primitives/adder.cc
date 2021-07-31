#include "drake/systems/primitives/adder.h"

#include "drake/common/default_scalars.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

template <typename T>
Adder<T>::Adder(int num_inputs, int size)
    : LeafSystem<T>(SystemTypeTag<Adder>{}) {
  for (int i = 0; i < num_inputs; i++) {
    this->DeclareInputPort(kUseDefaultName, kVectorValued, size);
  }

  this->DeclareVectorOutputPort("sum", size, &Adder<T>::CalcSum);
}

template <typename T>
template <typename U>
Adder<T>::Adder(const Adder<U>& other)
    : Adder<T>(other.num_input_ports(), other.get_input_port(0).size()) {}

template <typename T>
void Adder<T>::CalcSum(const Context<T>& context,
                       BasicVector<T>* sum) const {
  Eigen::VectorBlock<VectorX<T>> sum_vector = sum->get_mutable_value();

  // Zeroes the output.
  sum_vector.setZero();

  // Sum each input port into the output.
  for (int i = 0; i < context.num_input_ports(); i++) {
    sum_vector += this->get_input_port(i).Eval(context);
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Adder)
