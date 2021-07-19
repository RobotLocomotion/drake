#pragma once

#include <memory>

#include "drake/common/drake_assert.h"
#include "drake/common/value.h"

namespace drake {
namespace systems {

/// Makes a new AbstractValue containing the @p value.
template <typename T>
std::unique_ptr<AbstractValue> PackValue(T value) {
  return std::make_unique<Value<T>>(value);
}

/// Extracts data of type T from the given @p value, or throws if the
/// @p value does not contain type T.
template <typename T>
T UnpackValue(const AbstractValue& value) {
  return value.get_value<T>();
}

/// Extracts an integer from the given @p value, or aborts if the
/// @p value does not contain an integer.
int UnpackIntValue(const AbstractValue* value) {
  return UnpackValue<int>(*value);
}

/// Extracts an integer from the given @p value, or aborts if the
/// @p value does not contain an integer.
int UnpackIntValue(const AbstractValue& value) {
  return UnpackValue<int>(value);
}

}  // namespace systems
}  // namespace drake
