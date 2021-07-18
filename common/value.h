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
using ValueTraits = ValueTraitsImpl<T, std::is_copy_constructible_v<T>>;

// SFINAE type for whether Value<T>(Arg1, Args...) should be a forwarding ctor.
// In our ctor overload that copies into the storage, choose_copy == true.
template <bool choose_copy, typename T, typename Arg1, typename... Args>
using ValueForwardingCtorEnabled = typename std::enable_if_t<
  // There must be such a constructor.
  std::is_constructible_v<T, Arg1, Args...> &&
  // Disable this ctor when given T directly; in that case, we
  // should call our Value(const T&) ctor above, not try to copy-
  // construct a T(const T&).
  !std::is_same_v<T, Arg1> &&
  !std::is_same_v<T&, Arg1> &&
  // Only allow real ctors, not POD "constructor"s.
  !std::is_fundamental_v<T> &&
  // Disambiguate our copy implementation from our clone implementation.
  (choose_copy == std::is_copy_constructible_v<T>)>;

template <typename T>
using remove_cvref_t = std::remove_cv_t<std::remove_reference_t<T>>;

}  // namespace internal
#endif

/// A fully type-erased container class.  An AbstractValue stores an object of
/// some type T (where T is declared during at construction time) that at
/// runtime can be passed between functions without mentioning T.  Only when
/// the stored T must be accessed does the user need to mention T again.
///
/// (Advanced.) Note that AbstractValue's getters and setters method declare
/// that "If T does not match, a std::logic_error will be thrown with a helpful
/// error message".  The code that implements this check uses hashing, so in
/// the extraordinarily unlikely case of a 64-bit hash collision, the error may
/// go undetected in Release builds. (Debug builds have extra checks that will
/// trigger.)
///
/// (Advanced.) Only Value should inherit directly from AbstractValue.
/// User-defined classes with additional features may inherit from Value.
class AbstractValue {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AbstractValue)

  virtual ~AbstractValue();

  /// Constructs an AbstractValue using T's default constructor, if available.
  /// This is only available for T's that support default construction.
#if !defined(DRAKE_DOXYGEN_CXX)
  template <typename T,
            typename = typename std::enable_if_t<
                std::is_default_constructible_v<T>>>
#endif
  static std::unique_ptr<AbstractValue> Make();

  /// Returns an AbstractValue containing the given @p value.
  template <typename T>
  static std::unique_ptr<AbstractValue> Make(const T& value);

  /// Returns the value wrapped in this AbstractValue as a const reference.
  /// The reference remains valid only until this object is set or destroyed.
  /// @tparam T The originally declared type of this AbstractValue, e.g., from
  /// AbstractValue::Make<T>() or Value<T>::Value().  If T does not match, a
  /// std::logic_error will be thrown with a helpful error message.
  template <typename T>
  const T& get_value() const { return cast<T>().get_value(); }

  /// Returns the value wrapped in this AbstractValue as mutable reference.
  /// The reference remains valid only until this object is set or destroyed.
  /// @tparam T The originally declared type of this AbstractValue, e.g., from
  /// AbstractValue::Make<T>() or Value<T>::Value().  If T does not match, a
  /// std::logic_error will be thrown with a helpful error message.
  template <typename T>
  T& get_mutable_value() { return cast<T>().get_mutable_value(); }

  /// Sets the value wrapped in this AbstractValue.
  /// @tparam T The originally declared type of this AbstractValue, e.g., from
  /// AbstractValue::Make<T>() or Value<T>::Value().  If T does not match, a
  /// std::logic_error will be thrown with a helpful error message.
  template <typename T>
  void set_value(const T& v) { cast<T>().set_value(v); }

  /// Returns the value wrapped in this AbstractValue, if T matches the
  /// originally declared type of this AbstractValue.
  /// @tparam T The originally declared type of this AbstractValue, e.g., from
  /// AbstractValue::Make<T>() or Value<T>::Value().  If T does not match,
  /// returns nullptr.
  template <typename T>
  const T* maybe_get_value() const;

  /// Returns the mutable value wrapped in this AbstractValue, if T matches the
  /// originally declared type of this AbstractValue.
  /// @tparam T The originally declared type of this AbstractValue, e.g., from
  /// AbstractValue::Make<T>() or Value<T>::Value().  If T does not match,
  /// returns nullptr.
  template <typename T>
  T* maybe_get_mutable_value();

  /// Returns a copy of this AbstractValue.
  virtual std::unique_ptr<AbstractValue> Clone() const = 0;

  /// Copies the value in @p other to this value.  If other is not compatible
  /// with this object, a std::logic_error will be thrown with a helpful error
  /// message.
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
  std::string GetNiceTypeName() const;

 protected:
#if !defined(DRAKE_DOXYGEN_CXX)
  // Use a struct argument (instead of a bare size_t) so that no code
  // tries to convert a single-element numeric initializer_list to
  // a `const AbstractValue&`.  (This works around a bug in GCC 5.)
  struct Wrap { size_t value{}; };
  explicit AbstractValue(Wrap hash)
      : type_hash_(hash.value) {}
#endif

 private:
  template <typename T> bool is_maybe_matched() const;
  template <typename T> const Value<T>& cast() const;
  template <typename T> Value<T>& cast();
  template <typename T> [[noreturn]] void ThrowCastError() const;
  [[noreturn]] void ThrowCastError(const std::string&) const;

  // The TypeHash<T>::value supplied by the Value<T> constructor.
  const size_t type_hash_;
};

/// A container class for an arbitrary type T (with some restrictions).  This
/// class inherits from AbstractValue and therefore at runtime can be passed
/// between functions without mentioning T.
///
/// Example:
/// @code
/// void print_string(const AbstractValue& arg) {
///   const std::string& message = arg.get_value<std::string>();
///   std::cerr << message;
/// }
/// void meow() {
///   const Value<std::string> value("meow");
///   print_string(value);
/// }
/// @endcode
///
/// (Advanced.) User-defined classes with additional features may subclass
/// Value, but should take care to override Clone().
///
/// @tparam T Must be copy-constructible or cloneable. Must not be a pointer,
/// array, nor have const, volatile, or reference qualifiers.
template <typename T>
class Value : public AbstractValue {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Value)

  static_assert(
      std::is_same_v<T, internal::remove_cvref_t<T>>,
      "T should not have const, volatile, or reference qualifiers.");

  static_assert(
      !std::is_pointer_v<T> && !std::is_array_v<T>,
      "T cannot be a pointer or array.");

  /// Constructs a Value<T> using T's default constructor, if available.
  /// This is only available for T's that support default construction.
#if !defined(DRAKE_DOXYGEN_CXX)
  template <typename T1 = T,
            typename = typename std::enable_if_t<
                std::is_default_constructible_v<T1>>>
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
  /// The reference remains valid only until this object is set or destroyed.
  const T& get_value() const;

  /// Returns a mutable reference to the stored value.
  /// The reference remains valid only until this object is set or destroyed.
  T& get_mutable_value();

  /// Replaces the stored value with a new one.
  void set_value(const T& v);

  // AbstractValue overrides.
  std::unique_ptr<AbstractValue> Clone() const override;
  void SetFrom(const AbstractValue& other) override;
  const std::type_info& type_info() const final;
  const std::type_info& static_type_info() const final;

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

// Extracts a hash of the type `T` in a __PRETTY_FUNCTION__ templated on T.
//
// For, e.g., TypeHash<int> the pretty_func string `pretty` looks like this:
//  GCC   7.3: "... calc() [with T = int; size_t K = 16; ..."
//  Clang 6.0: "... calc() [T = int]"
//
// We grab the characters for T's type (e.g., "int") and hash them using FNV1a.
//  https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function
//
// The value of @p which_argument chooses the which pretty template argument to
// hash.  (Only one argument at a time is ever hashed.)  In the GCC example
// above, which_argument of 0 hashes "int" and 1 hashes "16".
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
// When @p discard_nested is true, then stopping at '<' means our success-return
// value will be true; if discard_nested is false then seeing any '<' is an
// error.  Thus, we can detect and fail-fast when our specializations for
// template parameters fail to match.
//
// When @p discard_cast is true, we will omit a leading cast-expression after
// the equals sign, e.g., when pretty looks like "... [with K = (MyEnum)0]".
// If we see any other open-paren than this possibly-skipped cast, then our
// success-return value will be false.  Thus, we can detect and fail-fast when
// our specializations for non-type template parameters fail to match, or when
// T's like function pointer signatures appear.
//
// Note that the compiler is required to inform us at compile-time if there are
// undefined operations in the below, such as running off the end of a string.
// Therefore, so as long as this function compiles, we know that `pretty` had
// at least something that looks like "T = ..." in it.
//
// Returns true on success / false on failure.
constexpr bool hash_template_argument_from_pretty_func(
    const char* pretty, int which_argument,
    bool discard_nested, bool discard_cast,
    FNV1aHasher* result) {
  // Advance to the desired template argument.  For example, if which_argument
  // is 0 and pretty == "... calc() [T = int]", then advance to the typename
  // after the "T = " so that the cursor `p` is pointing at the 'i' in "int".
  const char* p = pretty;
  for (int n = 0; n <= which_argument; ++n) {
    for (; (*p != '='); ++p) {}  // Advance to the '=' that we want.
    ++p;                         // Advance to ' '.
    ++p;                         // Advance to the typename we want.
  }

  // For enums, GCC 7's pretty says "(MyEnum)0" not "MyEnum::kFoo".  We'll strip
  // off the useless parenthetical.
  if (discard_cast && (*p == '(')) {
    for (; (*p != ')'); ++p) {}  // Advance to the ')'.
    ++p;
  }

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
    // GCC distinguishes between "<unnamed>" and "{anonymous}", while Clang does
    // not. Map "<unamed>" to "{anonymous}" for consistency and to avoid
    // confusion with nested types ("<>") below.
    const char* const unnamed_spelling = "<unnamed>";
    const char* unnamed_iter = unnamed_spelling;
    pretty_iter = p;
    for (; *unnamed_iter != 0 && *pretty_iter != 0;
         ++unnamed_iter, ++pretty_iter) {
      if (*unnamed_iter != *pretty_iter) {
        break;
      }
    }
    if (*unnamed_iter == 0) {
      const char* const anonymous_spelling = "{anonymous}";
      for (const char* c = anonymous_spelling; *c; ++c) {
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
        return false;
      }
    }

    // If the type has parens (such as a function pointer or std::function),
    // then we can't handle it.  Adding support for function types involves not
    // only unpacking the return and argument types, but also adding support
    // for const / volatile / reference / etc.).
    if (*p == '(') {
      return false;
    }

    result->add_byte(*p);
    ++p;
  }

  return true;
}

// Akin to C++17 std::void_t<>.
template <typename...>
using typehasher_void_t = void;

// Traits type to ask whether T::NonTypeTemplateParameter exists.
template <typename T, typename U = void>
struct TypeHasherHasNonTypeTemplateParameter {
  static constexpr bool value = false;
};
template <typename T>
struct TypeHasherHasNonTypeTemplateParameter<
    T, typehasher_void_t<typename T::NonTypeTemplateParameter>> {
  static constexpr bool value = true;
};

// Provides a struct templated on T so that __PRETTY_FUNCTION__ will express T
// at compile time.  The calc() function feeds the string representation of T
// to `result`.  Returns true on success / false on failure.  This base struct
// handles non-templated values (e.g., int); in a specialization down below, we
// handle template template T's.
template <typename T, bool = TypeHasherHasNonTypeTemplateParameter<T>::value>
struct TypeHasher {
  // Returns true on success / false on failure.
  static constexpr bool calc(FNV1aHasher* result) {
    // With discard_nested disabled here, the hasher will fail if it sees a
    // '<' in the typename.  If that happens, it means that the parameter pack
    // specialization(s) below did not match as expected.
    const int which_argument = 0;
    const bool discard_nested = false;
    const bool discard_cast = false;
    return hash_template_argument_from_pretty_func(
        __PRETTY_FUNCTION__, which_argument,
        discard_nested, discard_cast, result);
  }
};

// Provides a struct templated on Ts... with a calc() that hashes a sequence of
// types (a template parameter pack).
template <typename... Args>
struct ParameterPackHasher {};
// Specializes for base case: an empty pack.
template <>
struct ParameterPackHasher<> {
  static constexpr bool calc(FNV1aHasher*) { return true; }
};
// Specializes for inductive case: recurse using first + rest.
template <typename A, typename... B>
struct ParameterPackHasher<A, B...> {
  static constexpr bool calc(FNV1aHasher* result) {
    bool success = TypeHasher<A>::calc(result);
    if (sizeof...(B)) {
      // Add delimiter so that pair<cub,scone> and pair<cubs,cone> are distinct.
      result->add_byte(',');
      success = success && ParameterPackHasher<B...>::calc(result);
    }
    return success;
  }
};

// Specializes TypeHasher for template types T so that we have the typename of
// each template argument separately from T's outer type (as explained in the
// overview above).
template <template <typename...> class T, class... Args>
struct TypeHasher<T<Args...>, false> {
  static constexpr bool calc(FNV1aHasher* result) {
    // First, hash just the "T" template template type, not the "<Args...>".
    const int which_argument = 0;
    const bool discard_nested = true;
    const bool discard_cast = false;
    bool success = hash_template_argument_from_pretty_func(
        __PRETTY_FUNCTION__, which_argument,
        discard_nested, discard_cast, result);
    // Then, hash the "<Args...>".  Add delimiters so that parameter pack
    // nesting is correctly hashed.
    result->add_byte('<');
    success = success && ParameterPackHasher<Args...>::calc(result);
    result->add_byte('>');
    return success;
  }
};

// Provides a struct templated on `Typename Konstant`, similar to TypeHasher<T>
// but here the "Konstant"'s string is hashed, not a typename.
template <typename T, T K>
struct ValueHasher {
  static constexpr bool calc(FNV1aHasher* result) {
    const int which_argument = 1;
    const bool discard_nested = false;
    const bool discard_cast = true;
    return hash_template_argument_from_pretty_func(
        __PRETTY_FUNCTION__, which_argument,
        discard_nested, discard_cast, result);
  }
};

// Specializes TypeHasher for a non-type template value so that we have the
// value of the template argument separately from T's outer type (as explained
// in the overview above).
template <typename T>
struct TypeHasher<T, true> {
  static constexpr bool calc(FNV1aHasher* result) {
    // First, hash just the "T" template template type, not the "<U u>".
    const int which_argument = 0;
    const bool discard_nested = true;
    const bool discard_cast = false;
    hash_template_argument_from_pretty_func(
        __PRETTY_FUNCTION__, which_argument,
        discard_nested, discard_cast, result);
    // Then, hash the "<U=u>".
    using U = typename T::NonTypeTemplateParameter::value_type;
    result->add_byte('<');
    bool success = TypeHasher<U>::calc(result);
    result->add_byte('=');
    success = success &&
        ValueHasher<U, T::NonTypeTemplateParameter::value>::calc(result);
    result->add_byte('>');
    return success;
  }
};

// Provides a struct templated on Ns... with a calc() that hashes a sequence of
// ints (an int parameter pack).
template <int... Ns>
struct IntPackHasher {};
// Specializes for base case: an empty pack.
template <>
struct IntPackHasher<> {
  static constexpr bool calc(FNV1aHasher*) { return true; }
};
// Specializes for inductive case: recurse using first + rest.
template <int N, int... Ns>
struct IntPackHasher<N, Ns...> {
  static constexpr bool calc(FNV1aHasher* result) {
    result->add_byte('i');
    result->add_byte('n');
    result->add_byte('t');
    result->add_byte('=');
    bool success = ValueHasher<int, N>::calc(result);
    if (sizeof...(Ns)) {
      result->add_byte(',');
      success = success && IntPackHasher<Ns...>::calc(result);
    }
    return success;
  }
};

// Specializes TypeHasher for Eigen-like types.
template <template <typename, int, int...> class T,
          typename U, int N, int... Ns>
struct TypeHasher<T<U, N, Ns...>, false> {
  static constexpr bool calc(FNV1aHasher* result) {
    // First, hash just the "T" template template type, not the "<U, N, Ns...>".
    const int which_argument = 0;
    const bool discard_nested = true;
    const bool discard_cast = false;
    bool success = hash_template_argument_from_pretty_func(
        __PRETTY_FUNCTION__, which_argument,
        discard_nested, discard_cast, result);
    // Then, hash the "<U, N, Ns...>".  Add delimiters so that parameter pack
    // nesting is correctly hashed.
    result->add_byte('<');
    success = success && TypeHasher<U>::calc(result);
    result->add_byte(',');
    success = success && IntPackHasher<N, Ns...>::calc(result);
    result->add_byte('>');
    return success;
  }
};

// Computes a typename hash as a compile-time constant.  By putting it into a
// static constexpr, we force the compiler to compute the hash at compile time.
//
// We use these compile-time hashes to improve the performance of the downcast
// checks in AbstractValue.  The hash constant ends up being inlined into the
// object code of AbstractValue's accessors.  (We cannot use `typeid(T).name()`
// for this purpose at compile-time, because it's not constexpr.)
//
// This implementation is intended to work for the kinds of `T`s we would see
// in a `Value<T>`; notably, it does not support `T`s of type `std::function`,
// function pointers, and the like.  It also does not support `T`'s with
// non-type template parameters.  Unsupported types yield a hash value of zero
// so that using-code can decide how to handle the failure.
template <typename T>
struct TypeHash {
  static constexpr size_t calc() {
    FNV1aHasher hasher;
    const bool success = TypeHasher<T>::calc(&hasher);
    const size_t hash = size_t(hasher);
    const size_t nonzero_hash = hash ? hash : 1;
    return success ? nonzero_hash : 0;
  }
  // The hash of "T", or zero when the type is not supported by the hasher.
  // (Such failures are expected to be rare.)
  static constexpr size_t value = calc();
};

// This is called once per process per T whose type_hash is 0.  It logs a
// TypeHash failure message to Drake's text log.
int ReportZeroHash(const std::type_info& detail);

// Any code in this file that uses TypeHash::value calls us for its T.
template <typename T, size_t hash>
struct ReportUseOfTypeHash {
  static void used() {
    // By default, do nothing.
  }
};
template <typename T>
struct ReportUseOfTypeHash<T, 0> {
  static void used() {
    static int dummy = ReportZeroHash(typeid(T));
    (void)(dummy);
  }
};

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
  static_assert(!std::is_same_v<T, std::remove_cv_t<AbstractValue>>,
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

template <typename T, typename>
std::unique_ptr<AbstractValue> AbstractValue::Make() {
  return std::unique_ptr<AbstractValue>(new Value<T>());
}

template <typename T>
std::unique_ptr<AbstractValue> AbstractValue::Make(const T& value) {
  return std::unique_ptr<AbstractValue>(new Value<T>(value));
}

template <typename T>
const T* AbstractValue::maybe_get_value() const {
  if (!is_maybe_matched<T>()) { return nullptr; }
  auto& self = static_cast<const Value<T>&>(*this);
  return &self.get_value();
}

template <typename T>
T* AbstractValue::maybe_get_mutable_value() {
  if (!is_maybe_matched<T>()) { return nullptr; }
  auto& self = static_cast<Value<T>&>(*this);
  return &self.get_mutable_value();
}

// In Debug mode, returns true iff `this` is-a `Value<T>`.  In Release mode, a
// false return means `this` is definitely not a `Value<T>`; true means `this`
// is-probably-a `Value<T>`, but might rarely appear even for mismatched types.
template <typename T>
bool AbstractValue::is_maybe_matched() const {
  constexpr auto hash = internal::TypeHash<T>::value;
  internal::ReportUseOfTypeHash<T, hash>::used();
  return (kDrakeAssertIsArmed || !hash) ? (typeid(T) == static_type_info()) :
      (hash == type_hash_);
}

// Casts this to a `const Value<T>&`, with error checking that throws.
template <typename T>
const Value<T>& AbstractValue::cast() const {
  if (!is_maybe_matched<T>()) { ThrowCastError<T>(); }
  return static_cast<const Value<T>&>(*this);
}

// Casts this to a `Value<T>&`, with error checking that throws.
template <typename T>
Value<T>& AbstractValue::cast() {
  if (!is_maybe_matched<T>()) { ThrowCastError<T>(); }
  return static_cast<Value<T>&>(*this);
}

// We use a separate method to report cast() errors so that cast() itself will
// be inlined in Release builds.
template <typename T>
void AbstractValue::ThrowCastError() const {
  ThrowCastError(NiceTypeName::Get<T>());
}

template <typename T>
template <typename T1, typename T2>
Value<T>::Value()
    : AbstractValue(Wrap{internal::TypeHash<T>::value}),
      value_{} {
  internal::ReportUseOfTypeHash<T, internal::TypeHash<T>::value>::used();
  Traits::reinitialize_if_necessary(&value_);
}

template <typename T>
Value<T>::Value(const T& v)
    : AbstractValue(Wrap{internal::TypeHash<T>::value}),
      value_(Traits::to_storage(v)) {
  internal::ReportUseOfTypeHash<T, internal::TypeHash<T>::value>::used();
}

// We construct-in-place into our Storage value_.
template <typename T>
template <typename Arg1, typename... Args, typename>
Value<T>::Value(Arg1&& arg1, Args&&... args)
    : AbstractValue(Wrap{internal::TypeHash<T>::value}),
      value_{std::forward<Arg1>(arg1), std::forward<Args>(args)...} {
  internal::ReportUseOfTypeHash<T, internal::TypeHash<T>::value>::used();
}

// We move a unique_ptr into our Storage value_.
template <typename T>
template <typename Arg1, typename... Args, typename, typename>
Value<T>::Value(Arg1&& arg1, Args&&... args)
    : AbstractValue(Wrap{internal::TypeHash<T>::value}),
      value_{std::make_unique<T>(
          std::forward<Arg1>(arg1), std::forward<Args>(args)...)} {
  internal::ReportUseOfTypeHash<T, internal::TypeHash<T>::value>::used();
}

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
    : AbstractValue(Wrap{internal::TypeHash<T>::value}),
      value_{Traits::to_storage(std::move(v))} {
  internal::ReportUseOfTypeHash<T, internal::TypeHash<T>::value>::used();
}

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
  value_ = Traits::to_storage(other.get_value<T>());
}

template <typename T>
const std::type_info& Value<T>::type_info() const {
  return typeid(get_value());
}

template <typename T>
const std::type_info& Value<T>::static_type_info() const {
  return typeid(T);
}

#endif  // DRAKE_DOXYGEN_CXX
}  // namespace drake
