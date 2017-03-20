#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <utility>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {

template <typename T>
class Value;

/// A fully type-erased container class.
///
/// Only Value<T> should inherit directly from AbstractValue. User-defined
/// classes that define additional type-erased features should inherit from
/// Value<T>.
///
/// Most methods offer two variants, e.g., SetFrom and SetFromOrThrow.  The
/// former variants are optimized to use static_casts in Release builds but
/// will not throw exceptions when concrete Value types are mismatched; the
/// latter variants are guaranteed to throw on mismatched types, but may be
/// slower at runtime.  Prefer using the faster version only in performance-
/// sensitive code (e.g., inner loops), and the safer version otherwise.
class AbstractValue {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AbstractValue)

  AbstractValue() {}
  virtual ~AbstractValue();

  /// Returns a copy of this AbstractValue.
  virtual std::unique_ptr<AbstractValue> Clone() const = 0;

  /// Copies the value in @p other to this value.
  /// In Debug builds, if the types don't match, std::bad_cast will be
  /// thrown.  In Release builds, this is not guaranteed.
  virtual void SetFrom(const AbstractValue& other) = 0;

  /// Like SetFrom, but throws on mismatched types even in Release builds.
  virtual void SetFromOrThrow(const AbstractValue& other) = 0;

  /// Returns the value wrapped in this AbstractValue, which must be of
  /// exactly type T.  T cannot be a superclass, abstract or otherwise.
  /// In Debug builds, if the types don't match, std::bad_cast will be
  /// thrown.  In Release builds, this is not guaranteed.
  ///
  /// TODO(david-german-tri): Once this uses static_cast under the hood in
  /// Release builds, lower-case it.
  template <typename T>
  const T& GetValue() const {
    return DownCastOrMaybeThrow<T>()->get_value();
  }

  /// Like GetValue, but throws on mismatched types even in Release builds.
  template <typename T>
  const T& GetValueOrThrow() const {
    return DownCastOrThrow<T>()->get_value();
  }

  /// Returns the value wrapped in this AbstractValue, which must be of
  /// exactly type T.  T cannot be a superclass, abstract or otherwise.
  /// In Debug builds, if the types don't match, std::bad_cast will be
  /// thrown.  In Release builds, this is not guaranteed.
  /// Intentionally not const: holders of const references to the AbstractValue
  /// should not be able to mutate the value it contains.
  template <typename T>
  T& GetMutableValue() {
    return DownCastMutableOrMaybeThrow<T>()->get_mutable_value();
  }

  /// Like GetMutableValue, but throws on mismatched types even in Release
  /// builds.
  template <typename T>
  T& GetMutableValueOrThrow() {
    return DownCastMutableOrThrow<T>()->get_mutable_value();
  }

  /// Sets the value wrapped in this AbstractValue, which must be of
  /// exactly type T.  T cannot be a superclass, abstract or otherwise.
  /// @param value_to_set The value to be copied into this AbstractValue.
  /// In Debug builds, if the types don't match, std::bad_cast will be
  /// thrown.  In Release builds, this is not guaranteed.
  template <typename T>
  void SetValue(const T& value_to_set) {
    DownCastMutableOrMaybeThrow<T>()->set_value(value_to_set);
  }

  /// Like SetValue, but throws on mismatched types even in Release builds.
  template <typename T>
  void SetValueOrThrow(const T& value_to_set) {
    DownCastMutableOrThrow<T>()->set_value(value_to_set);
  }

  /// Returns an AbstractValue containing the given @p value.
  template <typename T>
  static std::unique_ptr<AbstractValue> Make(const T& value) {
    return std::unique_ptr<AbstractValue>(new Value<T>(value));
  }

 private:
  // Casts this to a Value<T>*.  Throws std::bad_cast if the cast fails.
  template <typename T>
  Value<T>* DownCastMutableOrThrow() {
    // We cast away const in this private non-const method so that we can reuse
    // DownCastOrThrow. This is equivalent to duplicating the logic of
    // DownCastOrThrow with a non-const target type.
    return const_cast<Value<T>*>(DownCastOrThrow<T>());
  }

  // Casts this to a Value<T>*. In Debug builds, throws std::bad_cast if the
  // cast fails.
  template <typename T>
  Value<T>* DownCastMutableOrMaybeThrow() {
    // We cast away const in this private non-const method so that we can reuse
    // DownCastOrMaybeThrow. This is equivalent to duplicating the logic of
    // DownCastOrMaybeThrow with a non-const target type.
    return const_cast<Value<T>*>(DownCastOrMaybeThrow<T>());
  }

  // Casts this to a const Value<T>*. Throws std::bad_cast if the cast fails.
  template <typename T>
  const Value<T>* DownCastOrThrow() const {
    const Value<T>* value = dynamic_cast<const Value<T>*>(this);
    if (value == nullptr) {
      // This exception is commonly thrown when the AbstractValue does not
      // contain the concrete value type requested.
      throw std::bad_cast();
    }
    return value;
  }

  // Casts this to a const Value<T>*. In Debug builds, throws std::bad_cast if
  // the cast fails.
  template <typename T>
  const Value<T>* DownCastOrMaybeThrow() const {
    // TODO(david-german-tri): Use static_cast in Release builds for speed.
    return DownCastOrThrow<T>();
  }
};

/// A container class for an arbitrary type T. User-defined classes that
/// require additional type-erased features should subclass Value<T>, taking
/// care to define the copy constructors and override Clone().
/// @tparam T Must be copy-constructible and assignable.
template <typename T>
class Value : public AbstractValue {
 public:
  // Values are copyable but not moveable.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Value)

  /// Constructs a Value<T> using T's default constructor, if available.
  /// This is only available for T's that support default construction.
  ///
  /// @tparam T1 is template boilerplate; do not set.
  template <typename T1 = T,
            typename = typename std::enable_if<
                std::is_default_constructible<T1>::value>::type>
  Value() : value_{} {}

  /// Constructs a Value<T> by copying the given value @p v.
  explicit Value(const T& v) : value_(v) {}

  /// Constructs a Value<T> by forwarding the given @p args to T's constructor,
  /// if available.  This is only available for non-primitive T's that are
  /// constructible from @p args.
  template <typename... Args,
            typename = typename std::enable_if<
                std::is_constructible<T, Args...>::value &&
                !std::is_same<T, Args...>::value &&
                !std::is_same<T&, Args...>::value &&
                !std::is_fundamental<T>::value>::type>
  explicit Value(Args&&... args) : value_{std::forward<Args>(args)...} {}

  ~Value() override {}

  std::unique_ptr<AbstractValue> Clone() const override {
    return std::make_unique<Value<T>>(get_value());
  }

  void SetFrom(const AbstractValue& other) override {
    value_ = other.GetValue<T>();
  }

  void SetFromOrThrow(const AbstractValue& other) override {
    value_ = other.GetValueOrThrow<T>();
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

/// A container class for BasicVector<T>.
///
/// @tparam T The type of the vector data. Must be a valid Eigen scalar.
template <typename T>
class VectorValue : public Value<BasicVector<T>*> {
 public:
  explicit VectorValue(std::unique_ptr<BasicVector<T>> v)
      : Value<BasicVector<T>*>(v.get()), owned_value_(std::move(v)) {
    DRAKE_ASSERT_VOID(CheckInvariants());
  }

  ~VectorValue() override {}

  // VectorValues are copyable but not moveable.
  explicit VectorValue(const VectorValue& other)
      : Value<BasicVector<T>*>(nullptr) {
    if (other.get_value() != nullptr) {
      owned_value_ = other.get_value()->Clone();
      this->set_value(owned_value_.get());
    }
    DRAKE_ASSERT_VOID(CheckInvariants());
  }

  VectorValue& operator=(const VectorValue& other) {
    if (this == &other) {
      // Special case to do nothing, to avoid an unnecessary Clone.
    } else if (other.get_value() == nullptr) {
      owned_value_.reset();
      this->set_value(owned_value_.get());
    } else {
      owned_value_ = other.get_value()->Clone();
      this->set_value(owned_value_.get());
    }
    DRAKE_ASSERT_VOID(CheckInvariants());
    return *this;
  }

  std::unique_ptr<AbstractValue> Clone() const override {
    return std::make_unique<VectorValue>(*this);
  }

 private:
  void CheckInvariants() {
    DRAKE_DEMAND(owned_value_.get() == this->get_value());
  }

  std::unique_ptr<BasicVector<T>> owned_value_;
};

}  // namespace systems
}  // namespace drake
