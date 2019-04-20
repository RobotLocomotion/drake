#include "drake/systems/primitives/port_switch.h"

#include <utility>

#include "drake/common/default_scalars.h"

namespace drake {
namespace systems {

template <typename T>
PortSwitch<T>::PortSwitch(int vector_size) : PortSwitch(vector_size, nullptr) {}

template <typename T>
template <typename U>
PortSwitch<T>::PortSwitch(const PortSwitch<U>& other)
    : PortSwitch<T>(other.vector_size_, other.model_value_.get()) {
  for (int i = 1; i < other.num_input_ports(); i++) {
    DeclareInputPort(other.get_input_port(i).get_name());
  }
}

template <typename T>
PortSwitch<T>::PortSwitch(int vector_size, const AbstractValue* model_value)
    : LeafSystem<T>(SystemTypeTag<systems::PortSwitch>()),
      vector_size_(vector_size),
      model_value_(model_value ? model_value->Clone() : nullptr) {
  this->DeclareAbstractInputPort("port_selector", Value<InputPortIndex>{});
  if (vector_size_ >= 0) {
    DRAKE_DEMAND(!model_value);  // Vector OR AbstractValue.
    this->DeclareVectorOutputPort("value", BasicVector<T>(vector_size_),
                                  &PortSwitch<T>::CopyVectorOut);
  } else {
    DRAKE_DEMAND(model_value);  // Exactly one should be non-null.

    // Use the std::function<> overloads to work with `AbstractValue` type
    // directly and maintain type erasure.
    auto abstract_value_allocator = [this]() { return model_value_->Clone(); };
    namespace sp = std::placeholders;
    this->DeclareAbstractOutputPort(
        "value", abstract_value_allocator,
        std::bind(&PortSwitch<T>::CopyValueOut, this, sp::_1, sp::_2));
  }
}

template <typename T>
const InputPort<T>& PortSwitch<T>::DeclareInputPort(std::string name) {
  if (vector_size_ >= 0) {
    return LeafSystem<T>::DeclareInputPort(std::move(name), kVectorValued,
                                           vector_size_);
  }
  return this->DeclareAbstractInputPort(std::move(name), *model_value_);
}

template <typename T>
void PortSwitch<T>::CopyVectorOut(const Context<T>& context,
                                  BasicVector<T>* vector) const {
  const InputPortIndex* selector =
      this->template EvalInputValue<InputPortIndex>(context, 0);
  DRAKE_DEMAND(*selector >= 0 && *selector < this->num_input_ports());
  vector->SetFrom(*(this->EvalVectorInput(context, *selector)));
}

template <typename T>
void PortSwitch<T>::CopyValueOut(const Context<T>& context,
                                 AbstractValue* value) const {
  const InputPortIndex* selector =
      this->template EvalInputValue<InputPortIndex>(context, 0);
  DRAKE_DEMAND(*selector >= 0 && *selector < this->num_input_ports());
  value->SetFrom(
      this->get_input_port(*selector).template Eval<AbstractValue>(context));
}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::PortSwitch)
