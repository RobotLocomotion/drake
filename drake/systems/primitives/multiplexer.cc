#include "drake/systems/primitives/multiplexer.h"

#include <functional>
#include <numeric>

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

template <typename T>
Multiplexer<T>::Multiplexer(int num_scalar_inputs)
    : Multiplexer<T>(std::vector<int>(num_scalar_inputs, 1)) {}

template <typename T>
Multiplexer<T>::Multiplexer(std::vector<int> input_sizes)
    : Multiplexer<T>(input_sizes, BasicVector<T>(std::accumulate(
                                      input_sizes.begin(), input_sizes.end(), 0,
                                      std::plus<int>{}))) {}

template <typename T>
Multiplexer<T>::Multiplexer(const systems::BasicVector<T>& model_vector)
    : Multiplexer<T>(std::vector<int>(model_vector.size(), 1), model_vector) {}

template <typename T>
Multiplexer<T>::Multiplexer(std::vector<int> input_sizes,
                            const systems::BasicVector<T>& model_vector)
    : LeafSystem<T>(SystemTypeTag<systems::Multiplexer>{}),
      input_sizes_(input_sizes) {
  DRAKE_DEMAND(model_vector.size() == std::accumulate(input_sizes_.begin(),
                                                      input_sizes_.end(), 0,
                                                      std::plus<int>{}));
  for (const int input_size : input_sizes_) {
    this->DeclareInputPort(kVectorValued, input_size);
  }
  this->DeclareVectorOutputPort(model_vector,
                                &Multiplexer::CombineInputsToOutput);
}

template <typename T>
template <typename U>
Multiplexer<T>::Multiplexer(const Multiplexer<U>& other)
    : Multiplexer<T>(other.input_sizes_,
                     systems::BasicVector<T>(other.get_output_port(0).size())) {
}

template <typename T>
void Multiplexer<T>::CombineInputsToOutput(const Context<T>& context,
                                           BasicVector<T>* output) const {
  auto output_vector = output->get_mutable_value();
  int output_vector_index{0};
  for (int i = 0; i < this->get_num_input_ports(); ++i) {
    const int input_size = input_sizes_[i];
    output_vector.segment(output_vector_index, input_size) =
        this->EvalEigenVectorInput(context, i);
    output_vector_index += input_size;
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Multiplexer)
