#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <utility>

#include "ctti/type_id.hpp"

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/is_cloneable.h"
#include "drake/common/nice_type_name.h"

namespace drake {

#if !defined(DRAKE_DOXYGEN_CXX)
class AbstractValue;

template <typename T>
class Value;

// Declare some private helper structs.
namespace internal {

// Turns a type T into an integer hash.
template <typename T>
struct TypeHash {
  static constexpr std::uint64_t value = ctti::type_id<T>().hash();
};
template <typename T>
constexpr std::uint64_t TypeHash<T>::value;

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
                "T in Value<T> cannot be AbstractValue.");

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

}  // namespace internal
#endif

/// A fully type-erased container class.
///
/// Only Value<T> should inherit directly from AbstractValue. User-defined
/// classes that define additional type-erased features should inherit from
/// Value<T>.
class AbstractValue {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AbstractValue)

  virtual ~AbstractValue();

  /// Returns an AbstractValue containing the given @p value.
  template <typename T>
  static std::unique_ptr<AbstractValue> Make(const T& value) {
    return std::unique_ptr<AbstractValue>(new Value<T>(value));
  }

  /// Returns a copy of this AbstractValue.
  virtual std::unique_ptr<AbstractValue> Clone() const = 0;

  /// Returns the value wrapped in this AbstractValue, which must be of exactly
  /// type T (not a subclass).  If T is the wrong type, a std::logic_error will
  /// probably be thrown with a helpful error message (and is guaranteed to be
  /// thrown in Debug builds).
  template <typename T>
  const T& get_value() const {
    return down_cast_or_maybe_throw<T>()->get_value();
  }

  /// Returns the value wrapped in this AbstractValue, which must be of exactly
  /// type T (not a subclass).  If T is the wrong type, a std::logic_error will
  /// probably be thrown with a helpful error message (and is guaranteed to be
  /// thrown in Debug builds).
  template <typename T>
  T& get_mutable_value() {
    return down_cast_mutable_or_maybe_throw<T>()->get_mutable_value();
  }

  /// Sets the value wrapped in this AbstractValue, which must be of
  /// exactly type T.  T cannot be a superclass, abstract or otherwise.
  /// @param value_to_set The value to be copied or cloned into this
  /// AbstractValue. In Debug builds, if the types don't match,
  /// an std::logic_error will be thrown with a helpful error message. In
  /// Release builds, this is not guaranteed.
  template <typename T>
  void set_value(const T& value_to_set) {
    down_cast_mutable_or_maybe_throw<T>()->set_value(value_to_set);
  }

  /// Returns a pointer to the value wrapped in this AbstractValue.  If T is
  /// the wrong type, returns nullptr (even in Release builds).
  template <typename T>
  const T* maybe_get_value() const {
    if (!is_matched<T>()) { return nullptr; }
    auto* value = static_cast<const Value<T>*>(this);
    return &value->get_value();
  }

  /// Copies or clones the value in @p other to this value.  If other is not
  /// compatible with this type, a std::logic_error will probably be thrown
  /// with a helpful error message (and is guaranteed to be thrown in Debug
  /// builds).
  virtual void SetFrom(const AbstractValue& other) = 0;

  /// Returns typeid of the contained object of type T. If T is polymorphic,
  /// this returns the typeid of the most-derived type of the contained object.
  virtual const std::type_info& type_info() const = 0;

  /// Returns typeid(T) for this Value<T> object. If T is polymorphic, this
  /// does NOT reflect the typeid of the most-derived type of the contained
  /// object; the result is always the base type T.
  virtual const std::type_info& static_type_info() const = 0;

  /// Returns a human-readable name for the underlying type T. This may be
  /// slow but is useful for error messages. If T is polymorphic, this returns
  /// the typeid of the most-derived type of the contained object.
  std::string GetNiceTypeName() const {
    return NiceTypeName::Canonicalize(
        NiceTypeName::Demangle(type_info().name()));
  }

  // TODO(jwnimmer-tri) Deprecate me.
  /// Same as get_value.
  template <typename T>
  const T& GetValue() const {
    return get_value<T>();
  }
  // TODO(jwnimmer-tri) Deprecate me.
  /// Same as get_value but guaranteed to throw even in Release builds.
  template <typename T>
  const T& GetValueOrThrow() const {
    return down_cast_or_throw<T>()->get_value();
  }
  // TODO(jwnimmer-tri) Deprecate me.
  /// Same as maybe_get_value().
  template <typename T>
  const T* MaybeGetValue() const {
    return maybe_get_value<T>();
  }
  // TODO(jwnimmer-tri) Deprecate me.
  /// Same as get_mutable_value.
  template <typename T>
  T& GetMutableValue() {
    return get_mutable_value<T>();
  }
  // TODO(jwnimmer-tri) Deprecate me.
  /// Same as get_mutable_value but guaranteed to throw even in Release builds.
  template <typename T>
  T& GetMutableValueOrThrow() {
    return down_cast_mutable_or_throw<T>()->get_mutable_value();
  }
  // TODO(jwnimmer-tri) Deprecate me.
  /// Same as set_value.
  template <typename T>
  void SetValue(const T& value_to_set) {
    set_value(value_to_set);
  }
  // TODO(jwnimmer-tri) Deprecate me.
  /// Same as set_value but guaranteed to throw even in Release builds.
  template <typename T>
  void SetValueOrThrow(const T& value_to_set) {
    down_cast_mutable_or_throw<T>()->set_value(value_to_set);
  }
  // TODO(jwnimmer-tri) Deprecate me.
  /// Like set_from, but throws std::logic_error on mismatched types even in
  /// Release builds.
  void SetFromOrThrow(const AbstractValue& other) {
    if (this->static_type_info() != other.static_type_info()) {
      ThrowCastError(other.GetNiceTypeName());
    }
    SetFrom(other);
  }

 protected:
#if !defined(DRAKE_DOXYGEN_CXX)
  // Use a dummy argument so code never tries to convert an initializer_list to
  // an AbstractValue for methods that want a `const AbstractValue&`.  (This
  // works around a bug in GCC 5.)
  struct Passphrase {};
  explicit AbstractValue(std::uint64_t type_hash, Passphrase)
      : type_hash_(type_hash) {}
#endif

 private:
  // Returns true iff `this` is-a `Value<T>`.
  template <typename T>
  bool is_matched() const {
    return static_type_info() == typeid(T);
  }

  // Returns false if `this` is definitely not a `Value<T>`.
  // May return true spuriously.
  template <typename T>
  bool is_maybe_matched() const {
    return
        (internal::TypeHash<T>::value == type_hash_) &&
#ifdef DRAKE_ASSERT_IS_ARMED
        (this->static_type_info() == typeid(T));
#else
        true;
#endif
  }

  // Casts this to a const Value<T>*. Throws if T was wrong.
  template <typename T>
  const Value<T>* down_cast_or_throw() const {
    if (!is_matched<T>()) {
      ThrowCastError<T>();
    }
    return static_cast<const Value<T>*>(this);
  }

  // Casts this to a const Value<T>*. Best-effort throws if T was wrong.
  template <typename T>
  const Value<T>* down_cast_or_maybe_throw() const {
    if (!is_maybe_matched<T>()) {
      ThrowCastError<T>();
    }
    return static_cast<const Value<T>*>(this);
  }

  // Casts this to a Value<T>*. Throws if T was wrong.
  template <typename T>
  Value<T>* down_cast_mutable_or_throw() {
    if (!is_matched<T>()) {
      ThrowCastError<T>();
    }
    return static_cast<Value<T>*>(this);
  }

  // Casts this to a Value<T>*. Best-effort throws if T was wrong.
  template <typename T>
  Value<T>* down_cast_mutable_or_maybe_throw() {
    if (!is_maybe_matched<T>()) {
      ThrowCastError<T>();
    }
    return static_cast<Value<T>*>(this);
  }

  template <typename T>
  [[noreturn]] void ThrowCastError() const {
    ThrowCastError(NiceTypeName::Get<T>());
  }

  [[noreturn]] void ThrowCastError(const std::string& requested_type) const;

  const uint64_t type_hash_;
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
            typename = typename std::enable_if_t<
                std::is_default_constructible<T1>::value>>
#endif
  Value()
      : AbstractValue(internal::TypeHash<T>::value, Passphrase{}),
        value_{} {
    Traits::reinitialize_if_necessary(&value_);
  }

  /// Constructs a Value<T> by copying or cloning the given value @p v.
  explicit Value(const T& v)
      : AbstractValue(internal::TypeHash<T>::value, Passphrase{}),
        value_(Traits::to_storage(v)) {}

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
            typename = typename std::enable_if_t<
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
                internal::ValueTraits<T>::UseCopy::value
              >>
  explicit Value(Arg1&& arg1, Args&&... args)
      : AbstractValue(internal::TypeHash<T>::value, Passphrase{}),
        value_{std::forward<Arg1>(arg1), std::forward<Args>(args)...} {}

  // This overload is for cloneable T; we move a unique_ptr into our Storage.
  template <typename Arg1,
            typename... Args,
            typename = typename std::enable_if_t<
                // These predicates are the same as above ...
                std::is_constructible<T, Arg1, Args...>::value &&
                !std::is_same<T, Arg1>::value &&
                !std::is_same<T&, Arg1>::value &&
                !std::is_fundamental<T>::value &&
                // ... except only for cloneable T.
                !internal::ValueTraits<T>::UseCopy::value
              >,
            // Dummy to disambiguate this method from the above overload.
            typename = void>
  explicit Value(Arg1&& arg1, Args&&... args)
      : AbstractValue(internal::TypeHash<T>::value, Passphrase{}),
        value_{std::make_unique<T>(
            std::forward<Arg1>(arg1), std::forward<Args>(args)...)} {}
#endif

  /// Constructs a Value<T> by copying or moving the given value @p v.
  /// @pre v is non-null
  explicit Value(std::unique_ptr<T> v)
      : AbstractValue(internal::TypeHash<T>::value, Passphrase{}),
        value_{Traits::to_storage(std::move(v))} {}
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

  const std::type_info& static_type_info() const final {
    return typeid(T);
  }

  const std::type_info& type_info() const override {
    return typeid(get_value());
  }

  /// Returns a const reference to the stored value.
  const T& get_value() const { return Traits::access(value_); }

  /// Returns a mutable reference to the stored value.
  T& get_mutable_value() { return Traits::access(value_); }

  /// Replaces the stored value with a new one.
  void set_value(const T& v) { value_ = Traits::to_storage(v); }

 private:
  using Traits = internal::ValueTraits<T>;
  typename Traits::Storage value_;
};

}  // namespace drake
