#ifndef DRAKE_SYSTEMS_FRAMEWORK_VALUE_H_
#define DRAKE_SYSTEMS_FRAMEWORK_VALUE_H_

#include "drake/systems/framework/abstract_value.h"

namespace drake {
namespace systems {

/// A container class for an arbitrary type T.
template<typename T> class Value : public AbstractValue {
 public:
  // T must be copy-constructible.
  explicit Value(const T& v)
      : value_(v) {}

  explicit Value(const Value<T>& v) = default;

  ~Value() {}

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

#endif  // DRAKE_SYSTEMS_FRAMEWORK_VALUE_H_
