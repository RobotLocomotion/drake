#pragma once

/// @file
/// Template method implementations for constant_value_source.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/systems/primitives/constant_value_source.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/leaf_context.h"

namespace drake {
namespace systems {

template <typename T>
ConstantValueSource<T>::ConstantValueSource(
    std::unique_ptr<AbstractValue> value)
    : source_value_(std::move(value)) {
  this->DeclareAbstractOutputPort();
}

template <typename T>
std::unique_ptr<SystemOutput<T>> ConstantValueSource<T>::AllocateOutput(
    const Context<T>& context) const {
  std::unique_ptr<LeafSystemOutput<T>> output(new LeafSystemOutput<T>);
  output->add_port(source_value_->Clone());
  return std::unique_ptr<SystemOutput<T>>(output.release());
}

template <typename T>
void ConstantValueSource<T>::DoCalcOutput(const Context<T>& context,
                                        SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(System<T>::CheckValidOutput(output));
  DRAKE_ASSERT_VOID(System<T>::CheckValidContext(context));
  AbstractValue* output_data = output->GetMutableData(0);
  *output_data = *source_value_;
}

}  // namespace systems
}  // namespace drake
