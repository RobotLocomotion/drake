#include "drake/systems/primitives/multiplexer.h"

#include <functional>
#include <memory>

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

namespace {

// Given a Multiplexer templated on scalar type `U`, make a BasicVector of a
// different scalar type, `T`.  This is only supported when the output port is
// exactly typed as BasicVector (not a subclass of BasicVector) because
// BasicVector does not support scalar conversion in a way that preserves
// subtyping (see #5454 for some related discussion).
template <typename T, typename U>
std::unique_ptr<BasicVector<T>> MakeBasicVectorOrThrow(
    const Multiplexer<U>& other) {
  auto context = other.CreateDefaultContext();
  auto output = other.AllocateOutput(*context);
  const BasicVector<U>* old_vector = output->get_vector_data(0);
  DRAKE_THROW_UNLESS(typeid(*old_vector) == typeid(BasicVector<U>));
  return std::make_unique<BasicVector<T>>(old_vector->size());
}

}  // namespace

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
    : Multiplexer<T>(other.input_sizes_, *MakeBasicVectorOrThrow<T, U>(other)) {
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
