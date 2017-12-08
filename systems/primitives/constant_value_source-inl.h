#pragma once

/// @file
/// Template method implementations for constant_value_source.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

/* clang-format off to disable clang-format-includes */
#include "drake/systems/primitives/constant_value_source.h"
/* clang-format on */

#include <memory>
#include <utility>

namespace drake {
namespace systems {

template <typename T>
ConstantValueSource<T>::ConstantValueSource(
    std::unique_ptr<AbstractValue> value)
    : source_value_(std::move(value)) {
  // Use the "advanced" method to provide explicit non-member functors here
  // since we already have AbstractValues.
  this->DeclareAbstractOutputPort(
      [this](const Context<T>&) {
        return source_value_->Clone();
      },
      [this](const Context<T>&, AbstractValue* output) {
        output->SetFrom(*source_value_);
      });
}

}  // namespace systems
}  // namespace drake
