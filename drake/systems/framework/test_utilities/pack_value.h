#pragma once

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

/// Makes a new AbstractValue containing the @p value.
template <typename T>
std::unique_ptr<AbstractValue> PackValue(T value) {
  return std::unique_ptr<AbstractValue>(new Value<T>(value));
}

/// Extracts data of type T from the given @p value, or aborts if the
/// @p value does not contain type T.
template <typename T>
T UnpackValue(const AbstractValue& value) {
  const Value<T>* unpacked = dynamic_cast<const Value<T>*>(&value);
  DRAKE_DEMAND(unpacked != nullptr);
  return unpacked->get_value();
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
