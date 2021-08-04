#include "drake/systems/primitives/demultiplexer.h"

#include <numeric>

#include "drake/common/autodiff.h"
#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace systems {

template <typename T>
Demultiplexer<T>::Demultiplexer(const std::vector<int>& output_ports_sizes)
    : LeafSystem<T>(SystemTypeTag<Demultiplexer>{}),
      output_ports_sizes_(output_ports_sizes),
      output_ports_start_(CalcOutputPortsStart(output_ports_sizes)) {
  const int size =
      std::accumulate(output_ports_sizes.begin(), output_ports_sizes.end(), 0);

  this->DeclareInputPort(kUseDefaultName, kVectorValued, size);

  // TODO(david-german-tri): Provide a way to infer the type.
  const int num_output_ports = output_ports_sizes.size();
  DRAKE_THROW_UNLESS(num_output_ports >= 1);
  for (int i = 0; i < num_output_ports; ++i) {
    const int output_port_size = output_ports_sizes[i];
    // Require no zero size output port.
    DRAKE_THROW_UNLESS(output_port_size >= 1);
    this->DeclareVectorOutputPort(
        systems::kUseDefaultName, output_port_size,
        [this, i](const Context<T>& context, BasicVector<T>* vector) {
          this->CopyToOutput(context, OutputPortIndex(i), vector);
        });
  }
}

template <typename T>
Demultiplexer<T>::Demultiplexer(int size, int output_ports_size)
    : Demultiplexer(CalcOutputPortsSizes(size, output_ports_size)) {}

template <typename T>
template <typename U>
Demultiplexer<T>::Demultiplexer(const Demultiplexer<U>& other)
    : Demultiplexer<T>(other.get_output_ports_sizes()) {}

template <typename T>
void Demultiplexer<T>::CopyToOutput(const Context<T>& context,
                                    OutputPortIndex port_index,
                                    BasicVector<T>* output) const {
  const int out_size = this->get_output_port(port_index).size();
  const int out_start_index = output_ports_start_[port_index];

  // TODO(amcastro-tri): the output should simply reference the input port's
  // value to avoid copy.
  auto in_vector = this->get_input_port(0).Eval(context);
  auto out_vector = output->get_mutable_value();
  out_vector = in_vector.segment(out_start_index, out_size);
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Demultiplexer)
