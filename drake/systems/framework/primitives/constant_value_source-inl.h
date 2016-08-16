#pragma once

/// @file
/// Template method implementations for constant_value_source.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/systems/framework/primitives/constant_value_source.h"

#include <stdexcept>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace systems {

template <typename T>
ConstantValueSource<T>::ConstantValueSource(
    std::unique_ptr<AbstractValue> value)
    : source_value_(std::move(value)) {
  this->DeclareOutputPort(kAbstractValued, 0, kInheritedSampling);
}

template <typename T>
std::unique_ptr<SystemOutput<T>> ConstantValueSource<T>::AllocateOutput(
    const ContextBase<T>& context) const {
  std::unique_ptr<LeafSystemOutput<T>> output(new LeafSystemOutput<T>);
  output->get_mutable_ports()->push_back(
      std::make_unique<OutputPort>(source_value_->Clone()));
  return std::unique_ptr<SystemOutput<T>>(output.release());
}

template <typename T>
void ConstantValueSource<T>::EvalOutput(const ContextBase<T>& context,
                                        SystemOutput<T>* output) const {
  DRAKE_ASSERT(output->get_num_ports() == 1);
  AbstractValue* output_data = output->GetMutableData(0);
  *output_data = *source_value_;
}

}  // namespace systems
}  // namespace drake
