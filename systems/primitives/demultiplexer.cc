#include "drake/systems/primitives/demultiplexer.h"

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

template <typename T>
Demultiplexer<T>::Demultiplexer(int size, int output_ports_sizes)
    : LeafSystem<T>(SystemTypeTag<systems::Demultiplexer>{}) {
  // The size must be a multiple of output_ports_sizes.
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
template <typename U>
Demultiplexer<T>::Demultiplexer(const Demultiplexer<U>& other)
    : Demultiplexer<T>(other.get_input_port(0).size(),
                       other.get_output_port(0).size()) {}

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

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Demultiplexer)
