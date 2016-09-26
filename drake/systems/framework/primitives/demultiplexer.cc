#include "drake/systems/framework/primitives/demultiplexer.h"

#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template <typename T>
Demultiplexer<T>::Demultiplexer(int size) {
  // TODO(amcastro-tri): remove the size parameter from the constructor once
  // #3109 supporting automatic sizes is resolved.
  this->DeclareInputPort(kVectorValued, size, kInheritedSampling);
  // TODO(david-german-tri): Provide a way to infer the type.
  for (int i = 0; i < size; ++i) {
    this->DeclareOutputPort(kVectorValued, 1, kInheritedSampling);
  }
}

template <typename T>
void Demultiplexer<T>::EvalOutput(const Context<T>& context,
                                  SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));
  // TODO(amcastro-tri): the output should simply reference the input port's
  // value to avoid copy.
  auto in_vector = System<T>::EvalEigenVectorInput(context, 0);
  for (int iport = 0; iport < this->get_num_output_ports(); ++iport) {
    auto out_vector = System<T>::GetMutableOutputVector(output, iport);
    out_vector[0] = in_vector[iport];
  }
}

template class DRAKESYSTEMFRAMEWORK_EXPORT Demultiplexer<double>;
template class DRAKESYSTEMFRAMEWORK_EXPORT Demultiplexer<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
