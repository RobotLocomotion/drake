#ifndef DRAKE_SYSTEMS_FRAMEWORK_VALUE_H_
#define DRAKE_SYSTEMS_FRAMEWORK_VALUE_H_

#include "drake/systems/framework/abstract_value.h"

namespace drake {
namespace systems {

template<typename T> class Value : public AbstractValue {
 public:
  // T must be copy-constructable.
  explicit Value(const T& v)
      : value_(v) {}
  explicit Value(const Value<T>& v)
      : value_(v.value()) {}

  ~Value() {}

  virtual const T& value() const { return value_; }

  virtual T* mutable_value() { return &value_; }

  virtual void set_value(const T& v) { value_ = v; }

 private:
  T value_;
};

}  // namespace systems
}  // namespace drake

#endif  // DRAKE_SYSTEMS_FRAMEWORK_VALUE_H_