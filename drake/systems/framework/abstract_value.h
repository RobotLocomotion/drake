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

  /// Returns the value wrapped in this AbstractValue, which must be of type T.
  /// In Debug builds, if the types don't match, std::runtime_error will be
  /// thrown.  In Release builds, this is not guaranteed.
  template<typename T> const T& GetValue() const {
    return DownCastOrMaybeThrow<T>()->get_value();
  }

  /// Returns the value wrapped in this AbstractValue, which must be of type T.
  /// In Debug builds, if the types don't match, std::runtime_error will be
  /// thrown.  In Release builds, this is not guaranteed.
  /// Intentionally not const: holders of const references to the AbstractValue
  /// should not be able to mutate the value it contains.
  template<typename T> T& GetMutableValue() {
    return DownCastMutableOrMaybeThrow<T>()->get_mutable_value();
  }

  /// Sets the value wrapped in this AbstractValue, which must be of type T.
  /// In Debug builds, if the types don't match, std::runtime_error will be
  /// thrown.  In Release builds, this is not guaranteed.
  template<typename T> void SetValue(const T& value_to_set) {
    DownCastMutableOrMaybeThrow<T>()->set_value(value_to_set);
  }

 private:
  /// Casts this class to a Value<T>*. In Debug builds, throws
  /// std::runtime_error if the cast fails.
  /// TODO(david-german-tri): Use static_cast in Release builds for speed.
  template<typename T> Value<T>* DownCastMutableOrMaybeThrow() {
    Value<T>* value = dynamic_cast<Value<T>*>(this);
    ThrowIfNullptr(value);
    return value;
  }

  /// Casts this class to a const Value<T>*. In Debug builds, throws
  /// std::runtime_error if the cast fails.
  /// TODO(david-german-tri): Use static_cast in Release builds for speed.
  template<typename T> const Value<T>* DownCastOrMaybeThrow() const {
    const Value<T>* value = dynamic_cast<const Value<T>*>(this);
    ThrowIfNullptr(value);
    return value;
  }

  /// Throws an std::runtime_error if value is nullptr, indicating that
  /// the down-cast failed.
  template<typename T> void ThrowIfNullptr(const Value<T>* value) const {
    if (value == nullptr) {
      std::string error_prefix("Abstract value type was not ");
      throw std::runtime_error(error_prefix + GetTypeName<T>());
    }
  }

  // TODO(sherm1): Make this name more human-readable, along the lines of
  // NiceTypeName<T> in Simbody.
  template <typename T> static std::string GetTypeName() {
    return typeid(T).name();
  }
};


}  // namespace systems
}  // namespace drake

#endif  // DRAKE_SYSTEMS_FRAMEWORK_ABSTRACT_VALUE_H_
