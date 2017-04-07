#include "drake/systems/primitives/multiplexer.h"

#include <functional>
#include <numeric>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/eigen_autodiff_types.h"

namespace drake {
namespace systems {

template <typename T>
Multiplexer<T>::Multiplexer(int num_scalar_inputs)
    : Multiplexer<T>(std::vector<int>(num_scalar_inputs, 1)) {}

template <typename T>
Multiplexer<T>::Multiplexer(std::vector<int> input_sizes)
    : input_sizes_(input_sizes) {
  for (const int input_size : input_sizes_) {
    this->DeclareInputPort(kVectorValued, input_size);
  }
  const int output_size = std::accumulate(
      input_sizes.begin(), input_sizes.end(), 0, std::plus<int>{});
  this->DeclareVectorOutputPort(BasicVector<T>(output_size),
                                &Multiplexer::CombineInputsToOutput);
}

template <typename T>
Multiplexer<T>::Multiplexer(const systems::BasicVector<T>& model_vector)
    : input_sizes_(std::vector<int>(model_vector.size(), 1)) {
  for (int i = 0; i < model_vector.size(); ++i) {
    this->DeclareInputPort(kVectorValued, 1);
  }
  this->DeclareVectorOutputPort(model_vector,
                                &Multiplexer::CombineInputsToOutput);
}

template <typename T>
void Multiplexer<T>::CombineInputsToOutput(
    const Context<T>& context, BasicVector<T>* output) const {
  auto output_vector = output->get_mutable_value();
  int output_vector_index{0};
  for (int i = 0; i < this->get_num_input_ports(); ++i) {
    const int input_size = input_sizes_[i];
    output_vector.segment(output_vector_index, input_size) =
        this->EvalEigenVectorInput(context, i);
    output_vector_index += input_size;
  }
}

template class Multiplexer<double>;
template class Multiplexer<AutoDiffXd>;

}  // namespace systems
}  // namespace drake
