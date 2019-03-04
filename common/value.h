#pragma once

#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <utility>

#include "drake/common/copyable_unique_ptr.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/hash.h"
#include "drake/common/is_cloneable.h"
#include "drake/common/nice_type_name.h"

namespace drake {

#if !defined(DRAKE_DOXYGEN_CXX)
template <typename T>
class Value;

namespace internal {

// A traits type for Value<T>, where use_copy is true when T's copy constructor
// and copy-assignment operator are used and false when T's Clone is used.
template <typename T, bool use_copy>
struct ValueTraitsImpl {};
template <typename T>
using ValueTraits = ValueTraitsImpl<T, std::is_copy_constructible<T>::value>;

// SFINAE type for whether Value<T>(Arg1, Args...) should be a forwarding ctor.
// In our ctor overload that copies into the storage, choose_copy == true.
template <bool choose_copy, typename T, typename Arg1, typename... Args>
using ValueForwardingCtorEnabled = typename std::enable_if_t<
  // There must be such a constructor.
  std::is_constructible<T, Arg1, Args...>::value &&
  // Disable this ctor when given T directly; in that case, we
  // should call our Value(const T&) ctor above, not try to copy-
  // construct a T(const T&).
  !std::is_same<T, Arg1>::value &&
  !std::is_same<T&, Arg1>::value &&
  // Only allow real ctors, not POD "constructor"s.
  !std::is_fundamental<T>::value &&
  // Disambiguate our copy implementation from our clone implementation.
  (choose_copy == std::is_copy_constructible<T>::value)>;

}  // namespace internal
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

  virtual ~AbstractValue();

  /// Returns an AbstractValue containing the given @p value.
  template <typename T>
  static std::unique_ptr<AbstractValue> Make(const T& value);

  // TODO(david-german-tri): Once this uses static_cast under the hood in
  // Release builds, lower-case it.
  /// Returns the value wrapped in this AbstractValue, which must be of
  /// exactly type T.  T cannot be a superclass, abstract or otherwise.
  /// In Debug builds, if the types don't match, an std::logic_error will be
  /// thrown with a helpful error message. In Release builds, this is not
  /// guaranteed.
  template <typename T>
  const T& GetValue() const;

  /// Like GetValue, but throws std::logic_error on mismatched types even in
  /// Release builds.
  template <typename T>
  const T& GetValueOrThrow() const;

  /// Like GetValue, but quietly returns nullptr on mismatched types,
  /// even in Release builds.
  /// @returns A pointer to the stored value if T is the right type,
  ///          otherwise nullptr.
  template <typename T>
  const T* MaybeGetValue() const;

  /// Returns the value wrapped in this AbstractValue, which must be of
  /// exactly type T.  T cannot be a superclass, abstract or otherwise.
  /// In Debug builds, if the types don't match, an std::logic_error will be
  /// thrown with a helpful error message. In Release builds, this is not
  /// guaranteed. Intentionally not const: holders of const references to the
  /// AbstractValue should not be able to mutate the value it contains.
  template <typename T>
  T& GetMutableValue();

  /// Like GetMutableValue, but throws std::logic_error on mismatched types even
  /// in Release builds.
  template <typename T>
  T& GetMutableValueOrThrow();

  /// Sets the value wrapped in this AbstractValue, which must be of
  /// exactly type T.  T cannot be a superclass, abstract or otherwise.
  /// @param value_to_set The value to be copied or cloned into this
  /// AbstractValue. In Debug builds, if the types don't match,
  /// an std::logic_error will be thrown with a helpful error message. In
  /// Release builds, this is not guaranteed.
  template <typename T>
  void SetValue(const T& value_to_set);

  /// Like SetValue, but throws on mismatched types even in Release builds.
  template <typename T>
  void SetValueOrThrow(const T& value_to_set);

  /// Returns a copy of this AbstractValue.
  virtual std::unique_ptr<AbstractValue> Clone() const = 0;

  /// Copies or clones the value in @p other to this value.
  /// In Debug builds, if the types don't match, an std::logic_error will be
  /// thrown with a helpful error message. In Release builds, this is not
  /// guaranteed.
  virtual void SetFrom(const AbstractValue& other) = 0;

  /// Like SetFrom, but throws std::logic_error on mismatched types even in
  /// Release builds.
  virtual void SetFromOrThrow(const AbstractValue& other) = 0;

  /// Returns typeid of the contained object of type T. If T is polymorphic,
  /// this returns the typeid of the most-derived type of the contained object.
  virtual const std::type_info& type_info() const = 0;

  /// Returns typeid(T) for this Value<T> object. If T is polymorphic, this
  /// does NOT reflect the typeid of the most-derived type of the contained
  /// object; the result is always the base type T.
  const std::type_info& static_type_info() const { return static_type_info_; }

  /// Returns a human-readable name for the underlying type T. This may be
  /// slow but is useful for error messages. If T is polymorphic, this returns
  /// the typeid of the most-derived type of the contained object.
  std::string GetNiceTypeName() const;

 protected:
#if !defined(DRAKE_DOXYGEN_CXX)
  explicit AbstractValue(const std::type_info& static_type_info)
    : static_type_info_(static_type_info) {}
#endif

 private:
  template <typename T>
  bool is_matched() const;

  template <typename T>
  Value<T>* DownCastMutableOrThrow();

  template <typename T>
  Value<T>* DownCastMutableOrMaybeThrow();

  template <typename T>
  const Value<T>* DownCastOrThrow() const;

  template <typename T>
  const Value<T>* DownCastOrMaybeThrow() const;

  const std::type_info& static_type_info_;
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
  Value();

  /// Constructs a Value<T> by copying or cloning the given value @p v.
  explicit Value(const T& v);

  /// Constructs a Value<T> by forwarding the given @p args to T's constructor,
  /// if available.  This is only available for non-primitive T's that are
  /// constructible from @p args.
#if defined(DRAKE_DOXYGEN_CXX)
  template <typename... Args>
  explicit Value(Args&&... args);
#else
  // This overload is for copyable T.
  template <typename Arg1, typename... Args, typename =
      typename internal::ValueForwardingCtorEnabled<true, T, Arg1, Args...>>
  explicit Value(Arg1&& arg1, Args&&... args);
  // This overload is for cloneable T.
  template <typename Arg1, typename... Args, typename = void, typename =
      typename internal::ValueForwardingCtorEnabled<false, T, Arg1, Args...>>
  explicit Value(Arg1&& arg1, Args&&... args);
#endif

  /// Constructs a Value<T> by copying or moving the given value @p v.
  /// @pre v is non-null.
  explicit Value(std::unique_ptr<T> v);

  ~Value() override {}

  /// Returns a const reference to the stored value.
  const T& get_value() const;

  /// Returns a mutable reference to the stored value.
  T& get_mutable_value();

  /// Replaces the stored value with a new one.
  void set_value(const T& v);

  // AbstractValue overrides.
  std::unique_ptr<AbstractValue> Clone() const override;
  void SetFrom(const AbstractValue& other) override;
  void SetFromOrThrow(const AbstractValue& other) override;
  const std::type_info& type_info() const override;

  // A using-declaration adds these methods into our class's Doxygen.
  using AbstractValue::static_type_info;
  using AbstractValue::GetNiceTypeName;

 private:
  using Traits = internal::ValueTraits<T>;
  typename Traits::Storage value_;
};

#if !defined(DRAKE_DOXYGEN_CXX)
// Declare some private helper structs.
namespace internal {

// Work around a GCC 5 bug that forbids throw except in return statements.
constexpr int pretty_func_parse_error(bool is_error) {
  return is_error ? throw std::logic_error("Parse error") : 0;
}

// Extracts a hash of the type `T` in a __PRETTY_FUNCTION__ templated on T.
//
// For, e.g., TypeHash<int> the pretty_func string `pretty` looks like this:
//  GCC   7.3: "... calc() [with T = int; size_t = ..."
//  Clang 6.0: "... calc() [T = int]"
//
// We grab the characters for T's type (e.g., "int") and hash them using FNV1a.
//  https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
//
// If T is a template type like "std::vector<U>", we only hash "std::vector"
// here.  We stop when we reach a '<' because each template argument is hashed
// separately below using parameter packs (see `TypeHasher<T<Args...>>`).  This
// avoids compiler bugs where __PRETTY_FUNCTION__ is fickle about the spelling
// of "T = std::vector<U>" vs "T = std::vector<U, std::allocator<U>>", varying
// it from one method to the next.  Because we visit each base type in turn, we
// hash "std::vector" then "U" then "std::allocator" then "U" and so it doesn't
// matter exactly how templates end up being spelled in __PRETTY_FUNCTION__.
//
// Note that the compiler is required to inform us at compile-time if there are
// undefined operations in the below, such as running off the end of a string.
// Therefore, so as long as this function compiles, we know that `pretty` had
// at least something that looks like "T = ..." in it.
constexpr void hash_template_argument_from_pretty_func(
    const char* pretty, bool discard_nested, FNV1aHasher* result) {
  // Advance to the typename after the "T = ".
  const char* p = pretty;
  for (; (*p != '='); ++p) {}  // Advance to '='.
  ++p;                         // Advance to ' '.
  ++p;                         // Advance to the typename we want.

  // Hash the characters in the typename, ending either when the typename ends
  // (';' or ']') or maybe when the first template argument begins ('<').
  while ((*p != ';') && (*p != ']')) {
    // Map Clang's spelling for the anonymous namespace to match GCC.  Start by
    // searching for the clang spelling starting at `p` ...
    const char* const clang_spelling = "(anonymous namespace)";
    const char* clang_iter = clang_spelling;
    const char* pretty_iter = p;
    for (; *clang_iter != 0 && *pretty_iter != 0; ++clang_iter, ++pretty_iter) {
      if (*clang_iter != *pretty_iter) { break; }
    }
    // ... and if we found the entire clang_spelling, emit gcc_spelling instead.
    if (*clang_iter == 0) {
      const char* const gcc_spelling = "{anonymous}";
      for (const char* c = gcc_spelling; *c; ++c) {
        result->add_byte(*c);
      }
      p = pretty_iter;
      continue;
    }

    // If we find a nested type ("<>"), then either we expected it (in which
    // case we're done) or we didn't expect it (and something is wrong).
    if (*p == '<') {
      if (discard_nested) {
        break;
      } else {
        pretty_func_parse_error(true);
      }
    }

    // If the type has parens (such as a function pointer or std::function),
    // then we can't handle it.  Add support for function types involves not
    // only unpacking the return and argument types, but also adding support
    // for const / volatile / reference / etc.).
    if (*p == '(') {
      pretty_func_parse_error(true);
    }

    result->add_byte(*p);
    ++p;
  }
}

// Provides a struct templated on T so that __PRETTY_FUNCTION__ will express T
// at compile time.  The calc() function returns a hash of T.  This base struct
// handles non-templated values (e.g., int); in a specialization down below, we
// handle template template T's.
template <typename T>
struct TypeHasher {
  static constexpr void calc(FNV1aHasher* result) {
    // With discard_nested disabled here, the hasher will fail if it sees a
    // '<' in the typename.  If that happens, it means that the parameter pack
    // specialization(s) below did not match as expected.
    const bool discard_nested = false;
    hash_template_argument_from_pretty_func(
        __PRETTY_FUNCTION__, discard_nested, result);
  }
};

// Provides a struct templated on Ts... with a calc() that hashes a sequence of
// types (a template parameter pack).
template <typename... Args>
struct ParameterPackHasher {};
// Specializes for base case: an empty pack.
template <>
struct ParameterPackHasher<> { static constexpr void calc(FNV1aHasher*) {} };
// Specializes for inductive case: recurse using first + rest.
template <typename A, typename... B>
struct ParameterPackHasher<A, B...> {
  static constexpr void calc(FNV1aHasher* result) {
    TypeHasher<A>::calc(result);
    if (sizeof...(B)) {
      // Add delimiter so that pair<cub,scone> and pair<cubs,cone> are distinct.
      result->add_byte(',');
      ParameterPackHasher<B...>::calc(result);
    }
  }
};

// Specializes TypeHasher for template types T so that we have the typename of
// each template argument separately from T's outer type (as explained in the
// overview above).
template <template <typename...> class T, class... Args>
struct TypeHasher<T<Args...>> {
  static constexpr void calc(FNV1aHasher* result) {
    // First, hash just the "T" template template type, not the "<Args...>".
    const bool discard_nested = true;
    hash_template_argument_from_pretty_func(
        __PRETTY_FUNCTION__, discard_nested, result);
    // Then, hash the "<Args...>".  Add delimiters so that parameter pack
    // nesting is correctly hashed.
    result->add_byte('<');
    ParameterPackHasher<Args...>::calc(result);
    result->add_byte('>');
  }
};

// Provides a struct templated on an `int`, similar to TypeHasher<T> but where
// the template parameters are int(s), not typenames.
template <int N>
struct IntHasher {
  static constexpr void calc(FNV1aHasher* result) {
    const bool discard_nested = false;
    hash_template_argument_from_pretty_func(
        __PRETTY_FUNCTION__, discard_nested, result);
  }
};
// Provides a struct templated on Ns... with a calc() that hashes a sequence of
// ints (an int parameter pack).
template <int... Ns>
struct IntPackHasher {};
// Specializes for base case: an empty pack.
template <>
struct IntPackHasher<> { static constexpr void calc(FNV1aHasher*) {} };
// Specializes for inductive case: recurse using first + rest.
template <int N, int... Ns>
struct IntPackHasher<N, Ns...> {
  static constexpr void calc(FNV1aHasher* result) {
    IntHasher<N>::calc(result);
    if (sizeof...(Ns)) {
      result->add_byte(',');
      IntPackHasher<Ns...>::calc(result);
    }
  }
};

// Specializes TypeHasher for Eigen-like types.
template <template <typename, int, int...> class T,
          typename U, int N, int... Ns>
struct TypeHasher<T<U, N, Ns...>> {
  static constexpr void calc(FNV1aHasher* result) {
    // First, hash just the "T" template template type, not the "<U, N, Ns...>".
    const bool discard_nested = true;
    hash_template_argument_from_pretty_func(
        __PRETTY_FUNCTION__, discard_nested, result);
    // Then, hash the "<U, N, Ns...>".  Add delimiters so that parameter pack
    // nesting is correctly hashed.
    result->add_byte('<');
    TypeHasher<U>::calc(result);
    result->add_byte(',');
    IntPackHasher<N, Ns...>::calc(result);
    result->add_byte('>');
  }
};

// Computes a typename hash into a static constant.  By putting it into a
// static constexpr, we force the compiler to compute the hash at compile time.
// This implementation is intended to work for the kinds of `T`s we would see
// in a Value<T>; notably, it does not support T's of type std::function,
// function pointers, and the like.
template <typename T>
struct TypeHash {
  static constexpr size_t calc() {
    FNV1aHasher hasher;
    TypeHasher<T>::calc(&hasher);
    return size_t(hasher);
  }
  static constexpr size_t value = calc();
};
template <typename T>
constexpr size_t TypeHash<T>::value;

// For copyable types, we can store a T directly within Value<T> and we don't
// need any special tricks to create or retrieve it.
template <typename T>
struct ValueTraitsImpl<T, true> {
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

}  // namespace internal

template <typename T>
std::unique_ptr<AbstractValue> AbstractValue::Make(const T& value) {
  return std::unique_ptr<AbstractValue>(new Value<T>(value));
}

template <typename T>
const T& AbstractValue::GetValue() const {
  return DownCastOrMaybeThrow<T>()->get_value();
}

template <typename T>
const T& AbstractValue::GetValueOrThrow() const {
  return DownCastOrThrow<T>()->get_value();
}

template <typename T>
const T* AbstractValue::MaybeGetValue() const {
  if (!is_matched<T>()) { return nullptr; }
  auto* value = static_cast<const Value<T>*>(this);
  return &value->get_value();
}

template <typename T>
T& AbstractValue::GetMutableValue() {
  return DownCastMutableOrMaybeThrow<T>()->get_mutable_value();
}

template <typename T>
T& AbstractValue::GetMutableValueOrThrow() {
  return DownCastMutableOrThrow<T>()->get_mutable_value();
}

template <typename T>
void AbstractValue::SetValue(const T& value_to_set) {
  DownCastMutableOrMaybeThrow<T>()->set_value(value_to_set);
}

template <typename T>
void AbstractValue::SetValueOrThrow(const T& value_to_set) {
  DownCastMutableOrThrow<T>()->set_value(value_to_set);
}

// Returns true iff `this` is-a `Value<T>`.
template <typename T>
bool AbstractValue::is_matched() const {
  return typeid(T) == static_type_info_;
}

// Casts this to a Value<T>*. Throws if the cast fails.
template <typename T>
Value<T>* AbstractValue::DownCastMutableOrThrow() {
  // We cast away const in this private non-const method so that we can reuse
  // DownCastOrThrow. This is equivalent to duplicating the logic of
  // DownCastOrThrow with a non-const target type.
  return const_cast<Value<T>*>(DownCastOrThrow<T>());
}

// Casts this to a Value<T>*. In Debug builds, throws if the cast fails.
template <typename T>
Value<T>* AbstractValue::DownCastMutableOrMaybeThrow() {
  // We cast away const in this private non-const method so that we can reuse
  // DownCastOrMaybeThrow. This is equivalent to duplicating the logic of
  // DownCastOrMaybeThrow with a non-const target type.
  return const_cast<Value<T>*>(DownCastOrMaybeThrow<T>());
}

// Casts this to a const Value<T>*. Throws if the cast fails.
template <typename T>
const Value<T>* AbstractValue::DownCastOrThrow() const {
  if (!is_matched<T>()) {
    throw std::logic_error(
        "AbstractValue: a request to extract a value of type '" +
        NiceTypeName::Get<T>() + "' failed because the actual type was '" +
        GetNiceTypeName() + "'.");
  }
  return static_cast<const Value<T>*>(this);
}

// Casts this to a const Value<T>*. In Debug builds, throws if the cast fails.
template <typename T>
const Value<T>* AbstractValue::DownCastOrMaybeThrow() const {
  // TODO(david-german-tri): Use static_cast in Release builds for speed.
  return DownCastOrThrow<T>();
}

template <typename T>
template <typename T1, typename T2>
Value<T>::Value()
    : AbstractValue(typeid(T)),
      value_{} {
  Traits::reinitialize_if_necessary(&value_);
}

template <typename T>
Value<T>::Value(const T& v)
    : AbstractValue(typeid(T)),
      value_(Traits::to_storage(v)) {}

// We construct-in-place into our Storage value_.
template <typename T>
template <typename Arg1, typename... Args, typename>
Value<T>::Value(Arg1&& arg1, Args&&... args)
    : AbstractValue(typeid(T)),
      value_{std::forward<Arg1>(arg1), std::forward<Args>(args)...} {}

// We move a unique_ptr into our Storage value_.
template <typename T>
template <typename Arg1, typename... Args, typename, typename>
Value<T>::Value(Arg1&& arg1, Args&&... args)
    : AbstractValue(typeid(T)),
      value_{std::make_unique<T>(
          std::forward<Arg1>(arg1), std::forward<Args>(args)...)} {}

// An explanation of the this constructor:
//
// We start with a unique_ptr<T> v.  We std::move it to get an xvalue
// unique_ptr<T>&& that we pass to to_storage.
//
// In the copyable case, that matches to_storage(const unique_ptr<T>&), which
// does a nonnull check and then returns an alias to the owned const T& within
// v.  Back in the Value constructor, the value_ member constructor is offered
// const T& so it does T::T(const T&) copy construction.  As the constructor
// returns, the v argument goes out of scope and the T owned by v is deleted.
// The users's unique_ptr<T> was transferred to Value<T> with a single copy.
//
// In the cloneable case, that matches to_storage(unique_ptr<T>), which means v
// is moved into other. The to_storage does a nonnull check, then std::moves
// other into an xvalue unique_ptr<T>&& again, then constructs a
// copyable_unique_ptr<T> from the xvalue which moves the owned T resource into
// that temporary, then returns the temporary by-value.  By RVO, the return
// value was already directly place into value_ and we are done.  The user's
// unique_ptr<T> was transferred to Value<T> without any Clone.
template <typename T>
Value<T>::Value(std::unique_ptr<T> v)
    : AbstractValue(typeid(T)),
      value_{Traits::to_storage(std::move(v))} {}

template <typename T>
const T& Value<T>::get_value() const {
  return Traits::access(value_);
}

template <typename T>
T& Value<T>::get_mutable_value() {
  return Traits::access(value_);
}

template <typename T>
void Value<T>::set_value(const T& v) {
  value_ = Traits::to_storage(v);
}

template <typename T>
std::unique_ptr<AbstractValue> Value<T>::Clone() const {
  return std::make_unique<Value<T>>(get_value());
}

template <typename T>
void Value<T>::SetFrom(const AbstractValue& other) {
  value_ = Traits::to_storage(other.GetValue<T>());
}

template <typename T>
void Value<T>::SetFromOrThrow(const AbstractValue& other) {
  value_ = Traits::to_storage(other.GetValueOrThrow<T>());
}

template <typename T>
const std::type_info& Value<T>::type_info() const {
  return typeid(get_value());
}

#endif  // DRAKE_DOXYGEN_CXX
}  // namespace drake
