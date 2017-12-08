#pragma once

#include <memory>
#include <stdexcept>
#include <type_traits>
#include <typeinfo>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/is_cloneable.h"

namespace drake {
namespace systems {

#if !defined(DRAKE_DOXYGEN_CXX)
class AbstractValue;

template <typename T>
class Value;

// Declare some private helper structs.
namespace value_detail {

// A traits type for Value<T>, where use_copy is true when T's copy constructor
// and copy-assignment operator are used and false when T's Clone is used.
template <typename T, bool use_copy>
struct ValueTraitsImpl {};

// For copyable types, we can store a T directly within Value<T> and we don't
// need any special tricks to create or retrieve it.
template <typename T>
struct ValueTraitsImpl<T, true> {
  using UseCopy = std::true_type;
  using Storage = T;
  static void reinitialize_if_necessary(Storage*) {}
  static const T& to_storage(const T& other) { return other; }
  static const Storage& to_storage(const std::unique_ptr<T>& other) {
    DRAKE_DEMAND(other.get() != nullptr);
    return *other;
  }
  static const T& access(const Storage& storage) { return storage; }
  // NOLINTNEXTLINE(runtime/references)
  static T& access(Storage& storage) { return storage; }
};

// For non-copyable types, we store a copyable_unique_ptr<T> so that Value<T>'s
// implementation's use of operator= and such work naturally.  To store values,
// we must Clone them; to access values, we must de-reference the pointer.
template <typename T>
struct ValueTraitsImpl<T, false> {
  static_assert(
      drake::is_cloneable<T>::value,
      "Types placed into a Value<T> must either be copyable or cloneable");

  // We explicitly disallow Value<AbstractValue>.  In cases where it occurs, it
  // is likely that someone has created functions such as
  //   template DoBar(const AbstractValue& foo) { ... }
  //   template <class Foo> DoBar(const Foo& foo) { DoBar(Value<Foo>{foo}); }
  // and accidentally called DoBar<AbstractValue>, or similar mistakes.
  static_assert(!std::is_same<T, std::remove_cv<AbstractValue>::type>::value,
                "T in Value<T> cannnot be AbstractValue.");

  using UseCopy = std::false_type;
  using Storage = typename drake::copyable_unique_ptr<T>;
  static void reinitialize_if_necessary(Storage* value) {
    *value = std::make_unique<T>();
  }
  static Storage to_storage(const T& other) {
    return Storage{other.Clone()};
  }
  static Storage to_storage(std::unique_ptr<T> other) {
    DRAKE_DEMAND(other.get() != nullptr);
    return Storage{std::move(other)};
  }
  static const T& access(const Storage& storage) { return *storage; }
  // NOLINTNEXTLINE(runtime/references)
  static T& access(Storage& storage) { return *storage; }
};

template <typename T>
using ValueTraits = ValueTraitsImpl<T, std::is_copy_constructible<T>::value>;

}  // namespace value_detail
#endif

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

  /// Copies or clones the value in @p other to this value.
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
  /// @param value_to_set The value to be copied or cloned into this
  /// AbstractValue.
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
/// @tparam T Must be copy-constructible or cloneable.
template <typename T>
class Value : public AbstractValue {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Value)

  /// Constructs a Value<T> using T's default constructor, if available.
  /// This is only available for T's that support default construction.
#if !defined(DRAKE_DOXYGEN_CXX)
  // T1 is template boilerplate; do not specify it at call sites.
  template <typename T1 = T,
            typename = typename std::enable_if<
                std::is_default_constructible<T1>::value>::type>
#endif
  Value() : value_{} { Traits::reinitialize_if_necessary(&value_); }

  /// Constructs a Value<T> by copying or cloning the given value @p v.
  explicit Value(const T& v) : value_(Traits::to_storage(v)) {}

  /// Constructs a Value<T> by forwarding the given @p args to T's constructor,
  /// if available.  This is only available for non-primitive T's that are
  /// constructible from @p args.
#if defined(DRAKE_DOXYGEN_CXX)
  template <typename... Args>
  explicit Value(Args&&... args);
#else
  // This overload is for copyable T; we construct value_ in-place as Storage.
  template <typename Arg1,
            typename... Args,
            typename = typename std::enable_if<
                // There must be such a constructor.
                std::is_constructible<T, Arg1, Args...>::value &&
                // Disable this ctor when given T directly; in that case, we
                // should call our Value(const T&) ctor above, not try to copy-
                // construct a T(const T&).
                !std::is_same<T, Arg1>::value &&
                !std::is_same<T&, Arg1>::value &&
                // Only allow real ctors, not POD "constructor"s.
                !std::is_fundamental<T>::value &&
                // Use this only for copyable T's.
                value_detail::ValueTraits<T>::UseCopy::value
              >::type>
  explicit Value(Arg1&& arg1, Args&&... args)
      : value_{std::forward<Arg1>(arg1), std::forward<Args>(args)...} {}

  // This overload is for cloneable T; we move a unique_ptr into our Storage.
  template <typename Arg1,
            typename... Args,
            typename = typename std::enable_if<
                // These predicates are the same as above ...
                std::is_constructible<T, Arg1, Args...>::value &&
                !std::is_same<T, Arg1>::value &&
                !std::is_same<T&, Arg1>::value &&
                !std::is_fundamental<T>::value &&
                // ... except only for cloneable T.
                !value_detail::ValueTraits<T>::UseCopy::value
              >::type,
            // Dummy to disambiguate this method from the above overload.
            typename = void>
  explicit Value(Arg1&& arg1, Args&&... args)
      : value_{std::make_unique<T>(
            std::forward<Arg1>(arg1), std::forward<Args>(args)...)} {}
#endif

  /// Constructs a Value<T> by copying or moving the given value @p v.
  /// @pre v is non-null
  explicit Value(std::unique_ptr<T> v)
      : value_{Traits::to_storage(std::move(v))} {}
  // An explanation of the above constructor:
  //
  // We start with a unique_ptr<T> v.  We std::move it to get an xvalue
  // unique_ptr<T>&& that we pass to to_storage.
  //
  // In the copyable case, that matches to_storage(const unique_ptr<T>&), which
  // does a nonnull check and then returns an alias to the owned const T&
  // within v.  Back in the Value constructor, the value_ member constructor is
  // offered const T& so it does T::T(const T&) copy construction.  As the
  // constructor returns, the v argument goes out of scope and the T owned by v
  // is deleted.  The users's unique_ptr<T> was transferred to Value<T> with a
  // single copy.
  //
  // In the cloneable case, that matches to_storage(unique_ptr<T>), which means
  // v is moved into other. The to_storage does a nonnull check, then
  // std::moves other into an xvalue unique_ptr<T>&& again, then constructs a
  // copyable_unique_ptr<T> from the xvalue which moves the owned T resource
  // into that temporary, then returns the temporary by-value.  By RVO, the
  // return value was already directly place into value_ and we are done.  The
  // user's unique_ptr<T> was transferred to Value<T> without any Clone.

  ~Value() override {}

  std::unique_ptr<AbstractValue> Clone() const override {
    return std::make_unique<Value<T>>(get_value());
  }

  void SetFrom(const AbstractValue& other) override {
    value_ = Traits::to_storage(other.GetValue<T>());
  }

  void SetFromOrThrow(const AbstractValue& other) override {
    value_ = Traits::to_storage(other.GetValueOrThrow<T>());
  }

  /// Returns a const reference to the stored value.
  const T& get_value() const { return Traits::access(value_); }

  /// Returns a mutable reference to the stored value.
  T& get_mutable_value() { return Traits::access(value_); }

  /// Replaces the stored value with a new one.
  void set_value(const T& v) { value_ = Traits::to_storage(v); }

 private:
  using Traits = value_detail::ValueTraits<T>;
  typename Traits::Storage value_;
};

}  // namespace systems
}  // namespace drake
