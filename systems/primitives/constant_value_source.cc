#include "drake/systems/primitives/constant_value_source.h"

namespace drake {
namespace systems {

template <typename T>
ConstantValueSource<T>::ConstantValueSource(const AbstractValue& value)
    : LeafSystem<T>(SystemTypeTag<ConstantValueSource>{}),
      source_value_(value.Clone()) {
  // Use the "advanced" method to provide explicit non-member functors here
  // since we already have AbstractValues.
  this->DeclareAbstractOutputPort(
      kUseDefaultName,
      [this]() {
        return source_value_->Clone();
      },
      [this](const Context<T>&, AbstractValue* output) {
        output->SetFrom(*source_value_);
      });
}

template <typename T>
template <typename U>
ConstantValueSource<T>::ConstantValueSource(const ConstantValueSource<U>& other)
    : ConstantValueSource<T>(*other.source_value_) {}

}  // namespace systems
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::systems::ConstantValueSource)
