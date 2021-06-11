#include "drake/systems/primitives/multiplexer.h"

#include <functional>
#include <memory>
#include <numeric>
#include <utility>

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

// N.B. This overload can support scalar conversion because our output port is
// definitely typed as BasicVector (and not some subtype of BasicVector).
template <typename T>
Multiplexer<T>::Multiplexer(int num_scalar_inputs)
    : Multiplexer<T>(std::vector<int>(num_scalar_inputs, 1)) {}

// N.B. This overload also supports scalar conversion.
template <typename T>
Multiplexer<T>::Multiplexer(std::vector<int> input_sizes)
    : Multiplexer<T>(
          SystemTypeTag<Multiplexer>{}, input_sizes,
          BasicVector<T>(std::accumulate(input_sizes.begin(), input_sizes.end(),
                                         0, std::plus<int>{}))) {}

// N.B. This overload cannot support scalar conversion, until we have a way to
// convert a BasicVector subtype between scalar types.
template <typename T>
Multiplexer<T>::Multiplexer(const systems::BasicVector<T>& model_vector)
    : Multiplexer<T>({} /* empty to disallow scalar conversion */,
                     std::vector<int>(model_vector.size(), 1), model_vector) {}

template <typename T>
Multiplexer<T>::Multiplexer(SystemScalarConverter converter,
                            std::vector<int> input_sizes,
                            const systems::BasicVector<T>& model_vector)
    : LeafSystem<T>(std::move(converter)), input_sizes_(input_sizes) {
  DRAKE_DEMAND(model_vector.size() == std::accumulate(input_sizes_.begin(),
                                                      input_sizes_.end(), 0,
                                                      std::plus<int>{}));
  for (const int input_size : input_sizes_) {
    this->DeclareInputPort(kUseDefaultName, kVectorValued, input_size);
  }
  this->DeclareVectorOutputPort(kUseDefaultName, model_vector,
                                &Multiplexer::CombineInputsToOutput);
}

template <typename T>
template <typename U>
Multiplexer<T>::Multiplexer(const Multiplexer<U>& other)
    : Multiplexer<T>(other.input_sizes_) {}

template <typename T>
void Multiplexer<T>::CombineInputsToOutput(const Context<T>& context,
                                           BasicVector<T>* output) const {
  auto output_vector = output->get_mutable_value();
  int output_vector_index{0};
  for (int i = 0; i < this->num_input_ports(); ++i) {
    const int input_size = input_sizes_[i];
    output_vector.segment(output_vector_index, input_size) =
        this->get_input_port(i).Eval(context);
    output_vector_index += input_size;
  }
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::Multiplexer)
