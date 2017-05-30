#include "drake/systems/primitives/demultiplexer.h"

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template <typename T>
Demultiplexer<T>::Demultiplexer(int size, int output_ports_sizes) {
  // size must be a multiple of output_ports_sizes.
  DRAKE_DEMAND(size % output_ports_sizes == 0);

  const int num_output_ports = size / output_ports_sizes;

  // TODO(amcastro-tri): remove the size parameter from the constructor once
  // #3109 supporting automatic sizes is resolved.
  this->DeclareInputPort(kVectorValued, size);
  // TODO(david-german-tri): Provide a way to infer the type.
  for (int i = 0; i < num_output_ports; ++i) {
    this->DeclareVectorOutputPort(
        BasicVector<T>(output_ports_sizes),
        [this, i](const Context<T>& context, BasicVector<T>* vector) {
          this->CopyToOutput(context, OutputPortIndex(i), vector);
        });
  }
}

template <typename T>
void Demultiplexer<T>::CopyToOutput(const Context<T>& context,
                                    OutputPortIndex port_index,
                                    BasicVector<T>* output) const {
  // All output ports have the same size as defined in the constructor.
  const int out_size = this->get_output_port(0).size();

  // TODO(amcastro-tri): the output should simply reference the input port's
  // value to avoid copy.
  auto in_vector = System<T>::EvalEigenVectorInput(context, 0);
  auto out_vector = output->get_mutable_value();
  out_vector = in_vector.segment(port_index * out_size, out_size);
}

template <typename T>
Demultiplexer<symbolic::Expression>* Demultiplexer<T>::DoToSymbolic() const {
  const int size = this->get_input_port(0).size();
  return new Demultiplexer<symbolic::Expression>(
      size, size / this->get_num_output_ports());
}

template class Demultiplexer<double>;
template class Demultiplexer<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
