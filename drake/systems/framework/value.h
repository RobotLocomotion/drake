#pragma once

#include <memory>

#include "drake/systems/framework/abstract_value.h"

namespace drake {
namespace systems {

/// A container class for an arbitrary type T.
/// @tparam T Must be copy-constructible.
template <typename T>
class Value : public AbstractValue {
 public:
  explicit Value(const T& v) : value_(v) {}

  // Values are copyable but not moveable.
  Value(const Value<T>& other) = default;
  Value& operator=(const Value<T>& other) = default;
  Value(Value<T>&& other) = delete;
  Value& operator=(Value<T>&& other) = delete;

  ~Value() override {}

  std::unique_ptr<AbstractValue> Clone() const override {
    return std::unique_ptr<Value<T>>(new Value<T>(*this));
  }

  /// Returns a const reference to the stored value.
  const T& get_value() const { return value_; }

  /// Returns a mutable reference to the stored value.
  T& get_mutable_value() { return value_; }

  /// Replaces the stored value with a new one.
  void set_value(const T& v) { value_ = v; }

 private:
  T value_;
};

}  // namespace systems
}  // namespace drake
