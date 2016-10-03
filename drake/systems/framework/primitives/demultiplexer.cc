#include "drake/systems/framework/primitives/demultiplexer.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template <typename T>
Demultiplexer<T>::Demultiplexer(int length, int output_ports_sizes) {
  // length must be a multiple of output_ports_sizes.
  DRAKE_DEMAND(length % output_ports_sizes == 0);

  const int num_output_ports = length / output_ports_sizes;

  // TODO(amcastro-tri): remove the length parameter from the constructor once
  // #3109 supporting automatic lengths is resolved.
  this->DeclareInputPort(kVectorValued, length, kInheritedSampling);
  // TODO(david-german-tri): Provide a way to infer the type.
  for (int i = 0; i < num_output_ports; ++i) {
    this->DeclareOutputPort(
        kVectorValued, output_ports_sizes, kInheritedSampling);
  }
}

template <typename T>
void Demultiplexer<T>::EvalOutput(const Context<T>& context,
                                  SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));

  // All output ports have the same size as defined in the constructor.
  const int out_size = this->get_output_port(0).get_size();

  // TODO(amcastro-tri): the output should simply reference the input port's
  // value to avoid copy.
  auto in_vector = System<T>::EvalEigenVectorInput(context, 0);
  for (int iport = 0; iport < this->get_num_output_ports(); ++iport) {
    auto out_vector = System<T>::GetMutableOutputVector(output, iport);
    out_vector = in_vector.segment(iport * out_size, out_size);
  }
}

template class DRAKESYSTEMFRAMEWORK_EXPORT Demultiplexer<double>;
template class DRAKESYSTEMFRAMEWORK_EXPORT Demultiplexer<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
