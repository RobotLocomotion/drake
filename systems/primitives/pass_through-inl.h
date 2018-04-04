#pragma once

/// @file
/// Template method implementations for pass_through.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

/* clang-format off to disable clang-format-includes */
#include "drake/systems/primitives/pass_through.h"
/* clang-format on */

#include <memory>
#include <utility>

namespace drake {
namespace systems {

// TODO(amcastro-tri): remove the vector_size parameter from the constructor
// once #3109 supporting automatic sizes is resolved.
template <typename T>
PassThrough<T>::PassThrough(
    int vector_size,
    std::unique_ptr<const AbstractValue> abstract_model_value)
    : LeafSystem<T>(SystemTypeTag<systems::PassThrough>()),
      abstract_model_value_(std::move(abstract_model_value)) {
  if (!is_abstract()) {
    DRAKE_DEMAND(vector_size != -1);
    BasicVector<T> model_value(vector_size);
    this->DeclareVectorInputPort(model_value);
    this->DeclareVectorOutputPort(
        model_value, &PassThrough::DoCalcVectorOutput);
  } else {
    DRAKE_DEMAND(vector_size == -1);
    // TODO(eric.cousineau): Remove value parameter from the constructor once
    // the equivalent of #3109 for abstract values is also resolved.
    this->DeclareAbstractInputPort(*abstract_model_value_);
    // Use the std::function<> overloads to work with `AbstractValue` type
    // directly and maintain type erasure.
    auto abstract_value_allocator = [this](const Context<T>&) {
      return abstract_model_value_->Clone();
    };
    namespace sp = std::placeholders;
    this->DeclareAbstractOutputPort(
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
  const BasicVector<T>& input = *this->EvalVectorInput(context, 0);
  DRAKE_ASSERT(input.size() == output->size());
  output->SetFrom(input);
}

template <typename T>
void PassThrough<T>::DoCalcAbstractOutput(const Context<T>& context,
                                          AbstractValue* output) const {
  DRAKE_ASSERT(is_abstract());
  const AbstractValue& input =
      *this->EvalAbstractInput(context, 0);
  output->SetFrom(input);
}

template <typename T>
optional<bool> PassThrough<T>::DoHasDirectFeedthrough(
    int input_port, int output_port) const {
  DRAKE_DEMAND(input_port == 0);
  DRAKE_DEMAND(output_port == 0);
  // By definition, a pass-through will have direct feedthrough, as the
  // output depends directly on the input.
  return true;
}

}  // namespace systems
}  // namespace drake
