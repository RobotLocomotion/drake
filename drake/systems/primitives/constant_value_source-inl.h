#pragma once

#include <memory>
#include <utility>

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
  this->DeclareAbstractOutputPort(*source_value_);
}

template <typename T>
void ConstantValueSource<T>::DoCalcOutput(const Context<T>& context,
                                        SystemOutput<T>* output) const {
  output->GetMutableData(0)->SetFrom(*source_value_);
}

}  // namespace systems
}  // namespace drake
