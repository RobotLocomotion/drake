#include "drake/systems/primitives/demultiplexer.h"

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

template <typename T>
Demultiplexer<T>::Demultiplexer(int size,
                                const std::vector<int>& output_ports_sizes)
    : LeafSystem<T>(SystemTypeTag<systems::Demultiplexer>{}),
      output_ports_sizes_(output_ports_sizes) {
  // Require the input port size should not be zero.
  DRAKE_DEMAND(size >= 1);

  this->DeclareInputPort(kVectorValued, size);

  // The total output ports sizes should be equal to the input port size.
  int total_output_size = 0;
  for (auto& output_port_i : output_ports_sizes) {
    total_output_size += output_port_i;
  }
  DRAKE_DEMAND(total_output_size == size);

  // TODO(david-german-tri): Provide a way to infer the type.
  const int num_output_ports = output_ports_sizes.size();
  for (int i = 0; i < num_output_ports; ++i) {
    this->DeclareVectorOutputPort(
        BasicVector<T>(output_ports_sizes[i]),
        [this, i](const Context<T>& context, BasicVector<T>* vector) {
          this->CopyToOutput(context, OutputPortIndex(i), vector);
        });
  }
}

template <typename T>
Demultiplexer<T>::Demultiplexer(int size, int output_ports_size)
    : Demultiplexer(size, CalOutputPortsSizes(size, output_ports_size)) {}

template <typename T>
template <typename U>
Demultiplexer<T>::Demultiplexer(const Demultiplexer<U>& other)
    : Demultiplexer<T>(other.get_input_port(0).size(),
                       other.get_output_ports_sizes()) {}

template <typename T>
void Demultiplexer<T>::CopyToOutput(const Context<T>& context,
                                    OutputPortIndex port_index,
                                    BasicVector<T>* output) const {
  int previous_ports_size = 0;
  for (int i = 0; i < port_index; i++) {
    previous_ports_size += this->get_output_port(i).size();
  }
  const int out_size = this->get_output_port(port_index).size();

  // TODO(amcastro-tri): the output should simply reference the input port's
  // value to avoid copy.
  auto in_vector = this->get_input_port(0).Eval(context);
  auto out_vector = output->get_mutable_value();
  out_vector = in_vector.segment(previous_ports_size, out_size);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Demultiplexer)
