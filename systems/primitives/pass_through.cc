#include "drake/systems/primitives/pass_through.h"

#include <utility>

namespace drake {
namespace systems {

// TODO(amcastro-tri): remove the vector_size parameter from the constructor
// once #3109 supporting automatic sizes is resolved.
template <typename T>
PassThrough<T>::PassThrough(
    int vector_size,
    std::unique_ptr<const AbstractValue> abstract_model_value)
    : LeafSystem<T>(SystemTypeTag<PassThrough>()),
      abstract_model_value_(std::move(abstract_model_value)) {
  if (!is_abstract()) {
    DRAKE_DEMAND(vector_size != -1);
    BasicVector<T> model_value(vector_size);
    input_port_ = &this->DeclareVectorInputPort(model_value);
    output_port_ = &this->DeclareVectorOutputPort(
        model_value, &PassThrough::DoCalcVectorOutput);
  } else {
    DRAKE_DEMAND(vector_size == -1);
    // TODO(eric.cousineau): Remove value parameter from the constructor once
    // the equivalent of #3109 for abstract values is also resolved.
    input_port_ = &this->DeclareAbstractInputPort(*abstract_model_value_);
    // Use the std::function<> overloads to work with `AbstractValue` type
    // directly and maintain type erasure.
    auto abstract_value_allocator = [this]() {
      return abstract_model_value_->Clone();
    };
    namespace sp = std::placeholders;
    output_port_ = &this->DeclareAbstractOutputPort(
        abstract_value_allocator,
        std::bind(&PassThrough::DoCalcAbstractOutput, this, sp::_1, sp::_2));
  }
}

template <typename T>
template <typename U>
PassThrough<T>::PassThrough(const PassThrough<U>& other)
    : PassThrough(other.is_abstract() ? -1 : other.get_input_port().size(),
                  other.is_abstract() ? other.abstract_model_value_->Clone()
                                      : nullptr) {}

template <typename T>
void PassThrough<T>::DoCalcVectorOutput(
      const Context<T>& context,
      BasicVector<T>* output) const {
  DRAKE_ASSERT(!is_abstract());
  const auto& input = get_input_port().Eval(context);
  DRAKE_ASSERT(input.size() == output->size());
  output->get_mutable_value() = input;
}

template <typename T>
void PassThrough<T>::DoCalcAbstractOutput(const Context<T>& context,
                                          AbstractValue* output) const {
  DRAKE_ASSERT(is_abstract());
  output->SetFrom(this->get_input_port().template Eval<AbstractValue>(context));
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::PassThrough)
