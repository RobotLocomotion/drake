#pragma once

#include "drake/systems/framework/value.h"

namespace drake {
namespace systems {

template <typename T>
std::unique_ptr<AbstractValue> PackValue(T value) {
  return std::unique_ptr<AbstractValue>(new Value<T>(value));
}

template <typename T>
T UnpackValue(const AbstractValue& value) {
  return dynamic_cast<const Value<T>*>(&value)->get_value();
}

int UnpackIntValue(const AbstractValue* value) {
  return UnpackValue<int>(*value);
}

int UnpackIntValue(const AbstractValue& value) {
  return UnpackValue<int>(value);
}

}  // namespace systems
}  // namespace drake
