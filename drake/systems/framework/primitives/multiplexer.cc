#include "drake/systems/framework/primitives/multiplexer.h"

#include <functional>
#include <numeric>

namespace drake {
namespace systems {

template <typename T>
Multiplexer<T>::Multiplexer(int num_scalar_inputs)
    : Multiplexer<T>(std::vector<int>(num_scalar_inputs, 1)) {}

template <typename T>
Multiplexer<T>::Multiplexer(std::vector<int> input_sizes)
    : input_sizes_(input_sizes) {
  for (const int input_size : input_sizes_) {
    this->DeclareInputPort(kVectorValued, input_size, kInheritedSampling);
  }
  const int output_size = std::accumulate(
      input_sizes.begin(), input_sizes.end(), 0, std::plus<int>{});
  this->DeclareOutputPort(kVectorValued, output_size, kInheritedSampling);
}

template <typename T>
void Multiplexer<T>::EvalOutput(const Context<T>& context,
                                SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));
  auto output_vector = System<T>::GetMutableOutputVector(output, 0);
  int output_vector_index{0};
  for (int i = 0; i < this->get_num_input_ports(); ++i) {
    const int input_size = input_sizes_[i];
    output_vector.segment(output_vector_index, input_size) =
        this->EvalEigenVectorInput(context, i);
    output_vector_index += input_size;
  }
}

template class DRAKE_EXPORT Multiplexer<double>;

}  // namespace systems
}  // namespace drake
