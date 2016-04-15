#ifndef DRAKE_SYSTEMS_FRAMEWORK_ABSTRACT_VALUE_H_
#define DRAKE_SYSTEMS_FRAMEWORK_ABSTRACT_VALUE_H_

#include <stdexcept>
#include <string>
#include <typeinfo>

namespace drake {
namespace systems {

template<typename T> class Value;

class AbstractValue {
 public:
  AbstractValue() {}
  virtual ~AbstractValue() {}

  /// Returns the value if this class can dynamic_cast to Value<T>.
  // Otherwise, asserts.
  template<typename T> const T& GetValue() const {
    const Value<T>* value = dynamic_cast<const Value<T>*>(this);
    ThrowIfNullptr(value);
    return value->value();
  }

  /// Returns the value if this class can dynamic_cast to Value<T>.
  /// Otherwise, asserts. Intentionally not const: holders of
  /// const references to the AbstractValue should not be able
  /// to mutate the value it contains.
  template<typename T> T* GetMutableValue() {
    Value<T>* value = dynamic_cast<Value<T>*>(this);
    ThrowIfNullptr(value);
    return value->mutable_value();
  }

  /// Sets the value if this class can dynamic_cast to Value<T>.
  /// Otherwise, asserts.
  template<typename T> void SetValue(const T& value_to_set) {
    Value<T>* value = dynamic_cast<Value<T>*>(this);
    ThrowIfNullptr(value);
    return value->set_value(value_to_set);
  }

 private:
  // Throws
  template<typename T> void ThrowIfNullptr(const Value<T>* value) const {
    if (value == nullptr) {
      std::string error("Abstract value type was not ");
      error += typeid(T).name();
      throw std::runtime_error(error);
    }
  }
};


}  // namespace systems
}  // namespace drake

#endif  // DRAKE_SYSTEMS_FRAMEWORK_ABSTRACT_VALUE_H_