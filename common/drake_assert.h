#pragma once

#include <array>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

/// @file
/// Provides Drake's assertion mechanics. They come in two flavors:
///   - Double checking developer assumptions
///      - DRAKE_ASSERT can be armed or disarmed independently from the
///        system-wide asserts.
///      - DRAKE_DEMAND is similar to DRAKE_ASSERT but is always armed.
///   - Testing user inputs
///     - DRAKE_THROW_UNLESS is similar to DRAKE_DEMAND, but uses exceptions
///       instead of `::abort()`.
/// These are intended to be used both within Drake and by other software.

#ifdef DRAKE_DOXYGEN_CXX
/// @p DRAKE_ASSERT(condition) is similar to the built-in @p assert(condition)
/// from the C++ system header `<cassert>`.  Unless Drake's assertions are
/// disarmed by the pre-processor definitions listed below, @p DRAKE_ASSERT
/// will evaluate @p condition and iff the value is false will trigger an
/// assertion failure with a message showing at least the condition text,
/// function name, file, and line.
///
/// By default, assertion failures will :abort() the program.  However, when
/// using the pydrake python bindings, assertion failures will instead throw a
/// C++ exception that causes a python SystemExit exception.
///
/// Assertions are enabled or disabled using the following pre-processor macros:
///
/// - If @p DRAKE_ENABLE_ASSERTS is defined, then @p DRAKE_ASSERT is armed.
/// - If @p DRAKE_DISABLE_ASSERTS is defined, then @p DRAKE_ASSERT is disarmed.
/// - If both macros are defined, then it is a compile-time error.
/// - If neither are defined, then NDEBUG governs assertions as usual.
///
/// This header will define exactly one of either @p DRAKE_ASSERT_IS_ARMED or
/// @p DRAKE_ASSERT_IS_DISARMED to indicate whether @p DRAKE_ASSERT is armed.
///
/// This header will define both `constexpr bool drake::kDrakeAssertIsArmed`
/// and `constexpr bool drake::kDrakeAssertIsDisarmed` globals.
///
/// One difference versus the standard @p assert(condition) is that the
/// @p condition within @p DRAKE_ASSERT is always syntax-checked, even if
/// Drake's assertions are disarmed.
///
/// Treat @p DRAKE_ASSERT like a statement -- it must always be used
/// in block scope, and must always be followed by a semicolon.
#define DRAKE_ASSERT(condition)
/// Like @p DRAKE_ASSERT, except that the expression must be void-valued; this
/// allows for guarding expensive assertion-checking subroutines using the same
/// macros as stand-alone assertions.
#define DRAKE_ASSERT_VOID(expression)
/// Evaluates @p condition and iff the value is false will trigger an assertion
/// failure with a message showing at least the condition text, function name,
/// file, and line.
#define DRAKE_DEMAND(condition)
/// Silences a "no return value" compiler warning by calling a function that
/// always raises an exception or aborts (i.e., a function marked noreturn).
/// Only use this macro at a point where (1) a point in the code is truly
/// unreachable, (2) the fact that it's unreachable is knowable from only
/// reading the function itself (and not, e.g., some larger design invariant),
/// and (3) there is a compiler warning if this macro were removed.  The most
/// common valid use is with a switch-case-return block where all cases are
/// accounted for but the enclosing function is supposed to return a value.  Do
/// *not* use this macro as a "logic error" assertion; it should *only* be used
/// to silence false positive warnings.  When in doubt, throw an exception
/// manually instead of using this macro.
#define DRAKE_UNREACHABLE()
/// Provides a convenient wrapper to throw an exception when a condition is
/// unmet.  This is similar to an assertion, but uses exceptions instead of
/// `::abort()`, and cannot be disabled.
#define DRAKE_THROW_UNLESS(condition, ...)
/// Evaluates `condition` and iff the value is false will throw an exception
/// with a message showing at least the condition text, function name, file, and
/// line.
///
/// The condition must not be a pointer, where we'd implicitly rely on its
/// nullness. Instead, always write out "!= nullptr" to be precise.
///
/// Correct: `DRAKE_THROW_UNLESS(foo != nullptr);`
/// Incorrect: `DRAKE_THROW_UNLESS(foo);`
///
/// Because this macro is intended to provide a useful exception message to
/// users, we should err on the side of extra detail about the failure. The
/// meaning of "foo" isolated within error message text does not make it clear
/// that a null pointer is the proximate cause of the problem.
///
/// In addition to the `condition`, up to four value expressions can be
/// provided. Each value expression and its value will be included in the error
/// message. For example:
///
///   DRAKE_THROW_UNLESS(x < 0, x);
///
/// Will include the the value of `x` in the message. If too many value
/// expressions are specified, this will most likely produce a compiler error
/// referencing "ENCODE_EACH".
///
/// Not all value expression types are supported. The currently supported set
/// shouldn't be considered *the* definitive set. If yours isn't supported, feel
/// free to submit a PR to add it (reaching out for help as appropriate).
#define DRAKE_DEREF(ptr)
/// Derferences a pointer, with null checking. If the provided pointer is null,
/// throws an exception. Otherwise, returns a reference to the object being
/// pointed to.
///
/// If the pointer points to a const type, a const reference is returned. If it
/// points to a non-const type, a non-const reference is returned.
///
/// It will typically appear in a class's constructor when it aliases a an
/// input parameter.
///
/// Example usage:
///
/// @code{cpp}
///
/// class Foo {
///  public:
///   Foo(const Bar* bar) : bar_(DRAKE_DEREF(bar)) {}
///  private:
///   const Bar& bar_;
/// };
///
/// @warning The pointer passed must be an l-value; do not pass in temporaries.
/// E.g., this includes function calls that return pointers and pointers to
/// arrays (&x[0]).
///
/// @endcode
#else  //  DRAKE_DOXYGEN_CXX

// Users should NOT set these; only this header should set them.
// clang-format off
#ifdef DRAKE_ASSERT_IS_ARMED
# error Unexpected DRAKE_ASSERT_IS_ARMED defined.
#endif
#ifdef DRAKE_ASSERT_IS_DISARMED
# error Unexpected DRAKE_ASSERT_IS_DISARMED defined.
#endif
// clang-format on

// Decide whether Drake assertions are enabled.
// clang-format off
#if defined(DRAKE_ENABLE_ASSERTS) && defined(DRAKE_DISABLE_ASSERTS)
# error Conflicting assertion toggles.
#elif defined(DRAKE_ENABLE_ASSERTS)
# define DRAKE_ASSERT_IS_ARMED
#elif defined(DRAKE_DISABLE_ASSERTS) || defined(NDEBUG)
# define DRAKE_ASSERT_IS_DISARMED
#else
# define DRAKE_ASSERT_IS_ARMED
#endif
// clang-format on

namespace drake {
namespace internal {
// Abort the program with an error message.
[[noreturn]] void Abort(const char* condition, const char* func,
                        const char* file, int line);
// Report an assertion failure; will either Abort(...) or throw.
[[noreturn]] void AssertionFailed(const char* condition, const char* func,
                                  const char* file, int line);
}  // namespace internal
namespace assert {
// Allows for specialization of how to bool-convert Conditions used in
// assertions, in case they are not intrinsically convertible.  See
// common/symbolic/expression/formula.h for an example use.  This is a public
// interface to extend; it is intended to be specialized by unusual Scalar
// types that require special handling.
template <typename Condition>
struct ConditionTraits {
  static constexpr bool is_valid = std::is_convertible_v<Condition, bool>;
  static bool Evaluate(const Condition& value) { return value; }
};
}  // namespace assert
}  // namespace drake

#define DRAKE_UNREACHABLE()                                            \
  ::drake::internal::Abort("Unreachable code was reached?!", __func__, \
                           __FILE__, __LINE__)

#define DRAKE_DEMAND(condition)                                              \
  do {                                                                       \
    typedef ::drake::assert::ConditionTraits<                                \
        typename std::remove_cv_t<decltype(condition)>>                      \
        Trait;                                                               \
    static_assert(Trait::is_valid, "Condition should be bool-convertible."); \
    static_assert(                                                           \
        !std::is_pointer_v<decltype(condition)>,                             \
        "When using DRAKE_DEMAND on a raw pointer, always write out "        \
        "DRAKE_DEMAND(foo != nullptr), do not write DRAKE_DEMAND(foo) "      \
        "and rely on implicit pointer-to-bool conversion.");                 \
    if (!Trait::Evaluate(condition)) {                                       \
      ::drake::internal::AssertionFailed(#condition, __func__, __FILE__,     \
                                         __LINE__);                          \
    }                                                                        \
  } while (0)

#ifdef DRAKE_ASSERT_IS_ARMED
// Assertions are enabled.
namespace drake {
constexpr bool kDrakeAssertIsArmed = true;
constexpr bool kDrakeAssertIsDisarmed = false;
}  // namespace drake
#define DRAKE_ASSERT(condition) DRAKE_DEMAND(condition)
#define DRAKE_ASSERT_VOID(expression)                                \
  do {                                                               \
    static_assert(std::is_convertible_v<decltype(expression), void>, \
                  "Expression should be void.");                     \
    expression;                                                      \
  } while (0)
#else
// Assertions are disabled, so just typecheck the expression.
namespace drake {
constexpr bool kDrakeAssertIsArmed = false;
constexpr bool kDrakeAssertIsDisarmed = true;
}  // namespace drake
#define DRAKE_ASSERT(condition)                                              \
  do {                                                                       \
    typedef ::drake::assert::ConditionTraits<                                \
        typename std::remove_cv_t<decltype(condition)>>                      \
        Trait;                                                               \
    static_assert(Trait::is_valid, "Condition should be bool-convertible."); \
    static_assert(                                                           \
        !std::is_pointer_v<decltype(condition)>,                             \
        "When using DRAKE_ASSERT on a raw pointer, always write out "        \
        "DRAKE_ASSERT(foo != nullptr), do not write DRAKE_ASSERT(foo) "      \
        "and rely on implicit pointer-to-bool conversion.");                 \
  } while (0)
#define DRAKE_ASSERT_VOID(expression)                              \
  static_assert(std::is_convertible_v<decltype(expression), void>, \
                "Expression should be void.")
#endif

namespace drake {
namespace internal {

// StringifyErrorDetailValue converts `value` into a string. Some types are
// given special treatment to ensure a specific format.
//
// This function plays an important role in the DRAKE_THROW_UNLESS macro. In
// order for *that* function to be safely inlined, the string conversion must
// not be exposed. This wrapper launders all such conversions, isolating them
// from the macro expansion.
//
// Note: the `requires` clause does not list all supported types. Further
// support is provided by overloads of this function in other files. To see the
// full list of supported types, search the codebase for those overloads.
// Furthermore, this explicitly only supports built-in and `std` types. For
// other types, an overload for that type should be included with the type
// definition (see, e.g., fmt_eigen.h). */
template <typename T>
std::string StringifyErrorDetailValue(const T& value)
  requires(std::is_same_v<T, float> || std::is_same_v<T, double> ||
           std::is_same_v<T, int> || std::is_same_v<T, std::size_t> ||
           std::is_same_v<T, std::string> ||
           std::is_same_v<T, std::string_view> ||
           std::is_same_v<T, const char*>);

// The collection of optional name-value pairs passed to DRAKE_THROW_UNLESS.
// The values are stored as their `std::string` representations.
//
// We support up to four such pairs. The defined pairs will be loaded in the
// front of the array, and the first null-valued const char* indicates the end
// of the list (if there are fewer than four).
struct ThrowValuesBuf {
  std::array<std::pair<const char*, std::string>, 4> values;
};

// Throw an error message.
[[noreturn]] void Throw(const char* condition, const char* func,
                        const char* file, int line,
                        const ThrowValuesBuf& buffer = {});

// Packages the optional name-value pairs passed to DRAKE_THROW_UNLESS into a
// buffer for Throw.
template <typename... NamesAndValues>
[[noreturn]] __attribute__((noinline, cold)) void ThrowWithValues(
    const char* condition, const char* func, const char* file, int line,
    NamesAndValues... names_and_values) {
  constexpr size_t N = sizeof...(names_and_values);
  static_assert(N % 2 == 0,
                "There should be an even number: up to 4 (name, value) pairs.");
  ThrowValuesBuf buffer;
  // This is a "constexpr for" loop for 0 <= I < N.
  auto name_and_value =
      std::forward_as_tuple(std::forward<NamesAndValues>(names_and_values)...);
  [&]<size_t... I>(std::integer_sequence<size_t, I...>&&) {
    (((buffer.values[I].first = std::get<2 * I>(name_and_value),
       buffer.values[I].second =
           StringifyErrorDetailValue(std::get<2 * I + 1>(name_and_value))),
      ...));
  }(std::make_index_sequence<N / 2>{});

  Throw(condition, func, file, line, buffer);
}

// The following infrastructure lets us iterate through a list of macro variadic
// arguments (up to length four).
// See https://codecraft.co/variadic-macros-tricks.html for explanation.

// Support up to *four* variadic arguments.
#define _GET_NTH_ARG(_1, _2, _3, _4, N, ...) N
// Macros for encoding a value expression into its value. _e is short for
// _encode.
#define _e_0(...)
#define _e_1(value) static_cast<const char*>(#value), value
#define _e_2(value, ...) \
  static_cast<const char*>(#value), value, _e_1(__VA_ARGS__)
#define _e_3(value, ...) \
  static_cast<const char*>(#value), value, _e_2(__VA_ARGS__)
#define _e_4(value, ...) \
  static_cast<const char*>(#value), value, _e_3(__VA_ARGS__)

#define ACCUMULATE(...)                                                 \
  /* NOLINTNEXTLINE(whitespace/comma) */                                \
  _GET_NTH_ARG(__VA_ARGS__ __VA_OPT__(, ) _e_4, _e_3, _e_2, _e_1, _e_0) \
  (__VA_ARGS__)

/* Dereferences a pointer, throwing an exception for null.

 The extraneous parameters are used to provide context in the thrown exception.

 Note: if a pointer to a const type is passed in, a const reference comes out,
 otherwise a non-const reference comes out. */
template <typename PtrType>
auto SafeDereference(PtrType& ptr, const char* condition, const char* func,
                     const char* file, int line) -> decltype(*ptr)
  requires std::is_reference_v<decltype(*ptr)>
{
  if (ptr == nullptr) {
    Throw(condition, func, file, line);
  }
  return *ptr;
}

}  // namespace internal
}  // namespace drake

#define DRAKE_THROW_UNLESS(condition, ...)                                    \
  do {                                                                        \
    typedef ::drake::assert::ConditionTraits<                                 \
        typename std::remove_cv_t<decltype(condition)>>                       \
        Trait;                                                                \
    static_assert(Trait::is_valid, "Condition should be bool-convertible.");  \
    static_assert(                                                            \
        !std::is_pointer_v<decltype(condition)>,                              \
        "When using DRAKE_THROW_UNLESS on a raw pointer, always write out "   \
        "DRAKE_THROW_UNLESS(foo != nullptr), do not write DRAKE_THROW_UNLESS" \
        "(foo) and rely on implicit pointer-to-bool conversion.");            \
    if (!Trait::Evaluate(condition)) {                                        \
      ::drake::internal::ThrowWithValues(                                     \
          #condition, __func__, __FILE__,                                     \
          __LINE__ __VA_OPT__(, ACCUMULATE(__VA_ARGS__)));                    \
    }                                                                         \
  } while (0)

#define DRAKE_DEREF(ptr)                                                \
  ::drake::internal::SafeDereference(ptr, #ptr " != nullptr", __func__, \
                                     __FILE__, __LINE__)

#endif  // DRAKE_DOXYGEN_CXX
