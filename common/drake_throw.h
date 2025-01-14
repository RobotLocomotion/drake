#pragma once

#include <array>
#include <string>
#include <type_traits>
#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/fmt.h"

// TODO(SeanCurtis-TRI) We should merge the contents of this file into
// `drake_assert.h`. This file would simply include that file for backwards
// compatibility.

/** @file
Provides a convenient wrapper to throw an exception when a condition is
unmet.  This is similar to an assertion, but uses exceptions instead of
::abort(), and cannot be disabled.
*/

namespace drake {
namespace internal {

/* Reports if `T::string() const` exists. */
template <typename T>
constexpr bool has_string(...) { return false; }

template <typename T>
constexpr std::enable_if_t<
    std::is_convertible_v<decltype(std::declval<const T>().string()),
                          std::string_view>,
    bool>
has_string(int) {
  return true;
}

/* Reports if `T::to_string() const` exists. */
template <typename T>
constexpr bool has_to_string(...) { return false; }

template <typename T>
constexpr std::enable_if_t<
    std::is_convertible_v<decltype(std::declval<const T>().to_string()),
                          std::string_view>,
    bool>
has_to_string(int) {
  return true;
}

/* StringifyValue converts `value` into a string. Some types are given special
 treatment to ensure a specific format. */
template <typename T>
std::string StringifyValue(const T& value) {
  if constexpr (std::is_floating_point_v<T>) {
    return fmt_floating_point(value);
  } else if constexpr (std::is_same_v<T, char>) {
    return fmt::format("'{}'", value);
  } else if constexpr (std::is_convertible_v<T, std::string_view>) {
    return fmt_debug_string(value);
  } else if constexpr (has_string<T>(0)) {
    return value.string();
  } else if constexpr (has_to_string<T>(0)) {
    return value.to_string();
  } else {
    return fmt::to_string(value);
  }
}

// The collection of possible name-value pairs passed to DRAKE_THROW_UNLESS.
struct ThrowValuesBuf {
  std::array<std::pair<const char*, std::string>, 4> values;
};

// Throw an error message.
[[noreturn]] void Throw(const char* condition, const char* func,
                        const char* file, int line,
                        const ThrowValuesBuf& buffer = {});

template <typename... ValuePairs>
[[noreturn]] __attribute__((noinline, cold)) void ThrowWithValues(
    const char* condition, const char* func, const char* file, int line,
    ValuePairs&&... pairs) {
  constexpr size_t N = sizeof...(pairs);
  static_assert(N % 2 == 0,
                "There should be an even number: up to 4 (name, value) pairs.");
  ThrowValuesBuf buffer;
  // This is a "constexpr for" loop for 0 <= I < N.
  auto pairs_tuple = std::forward_as_tuple(std::forward<ValuePairs>(pairs)...);
  [&]<size_t... I>(std::integer_sequence<size_t, I...> &&) {
    (((buffer.values[I].first = std::get<2 * I>(std::move(pairs_tuple)),
       buffer.values[I].second =
           StringifyValue(std::get<2 * I + 1>(std::move(pairs_tuple)))),
      ...));
  }
  (std::make_index_sequence<N / 2>{});

  Throw(condition, func, file, line, buffer);
}

/* The following infrastructure lets us iterate through a list of macro variadic
 arguments (up to length four).
 See https://codecraft.co/2014/11/25/variadic-macros-tricks/ for explanation.
*/

// Support up to *four* variadic arguments.
#define _GET_NTH_ARG(_1, _2, _3, _4, N, ...) N
// Macros for encoding a value expression into its value. _e is short for
// _encode.
#define _e_0(...)
#define _e_1(value) static_cast<const char*>(#value), value
#define _e_2(value, ...) static_cast<const char*>(#value), value, \
    _e_1(__VA_ARGS__)
#define _e_3(value, ...) static_cast<const char*>(#value), value, \
    _e_2(__VA_ARGS__)
#define _e_4(value, ...) static_cast<const char*>(#value), value, \
    _e_3(__VA_ARGS__)

#define ACCUMULATE(...)                                                    \
    /* NOLINTNEXTLINE(whitespace/comma) */                                 \
    _GET_NTH_ARG(__VA_ARGS__ __VA_OPT__(,) _e_4, _e_3, _e_2, _e_1, _e_0)   \
        (__VA_ARGS__)


}  // namespace internal
}  // namespace drake

/** Evaluates @p condition and iff the value is false will throw an exception
with a message showing at least the condition text, function name, file,
and line.

The condition must not be a pointer, where we'd implicitly rely on its
nullness. Instead, always write out "!= nullptr" to be precise.

Correct: `DRAKE_THROW_UNLESS(foo != nullptr);`
Incorrect: `DRAKE_THROW_UNLESS(foo);`

Because this macro is intended to provide a useful exception message to
users, we should err on the side of extra detail about the failure. The
meaning of "foo" isolated within error message text does not make it
clear that a null pointer is the proximate cause of the problem.

In addition to the @p condition, up to four value expressions can be provided.
Each value expression and its value will be included in the error message. For
example:

  DRAKE_THROW_UNLESS(x < 0, x);

Will include the the value of `x` in the message. If too many value expressions
are specified, this will most likely produce a compiler error referencing
"ENCODE_EACH".

The message values are converted to strings in one of three ways:

    - Call the `value.to_string()` method on the value expression (if valid).
    - Call the `value.string()` method on the value expression (if valid).
    - Use `fmt::format`.

Therefore, if there is no `to_string()` or `string()` method available, the
type passed must be compatible with fmt::to_string(). This compatibility can
be achieved in a number of ways.

For eigen types, this requires application of the fmt_eigen function. Such as:

  DRAKE_THROW_UNLESS(v.norm() > 0, fmt_eigen(v), v.norm());

This will include the value of the vector v as well as its norm in the error
message.

Similarly, some types require fmt_streamed() such as:

  DRAKE_THROW_UNLESS(my_enum == SomeEnum::kValue, fmt_streamed(my_enum));

DRAKE_THROW_UNLESS prints the error message with the `fmt_eigen` and
`fmt_streamed` wrappers stripped out.

`fmt_eigen` and `fmt_streamed` are both located in the `drake` namespace. If
you use DRAKE_THROW_UNLESS outside of the `drake` namespace, it is advisable to
alias the required fmt-compatibility function into your local usage (this helps
DRAKE_THROW_UNLESS strip out the wrapper for the error message).

@warning DRAKE_THROW_UNLESS can be called with or without value expressions.
It must not be invoked with value expressions in header files. If passing
value expressions, the invocation must be in a .cc file.  */
#define DRAKE_THROW_UNLESS(condition, ...)                                    \
  do {                                                                        \
    typedef ::drake::assert::ConditionTraits<                                 \
        typename std::remove_cv_t<decltype(condition)>> Trait;                \
    static_assert(Trait::is_valid, "Condition should be bool-convertible.");  \
    static_assert(                                                            \
        !std::is_pointer_v<decltype(condition)>,                              \
        "When using DRAKE_THROW_UNLESS on a raw pointer, always write out "   \
        "DRAKE_THROW_UNLESS(foo != nullptr), do not write DRAKE_THROW_UNLESS" \
        "(foo) and rely on implicit pointer-to-bool conversion.");            \
    if (!Trait::Evaluate(condition)) {                                        \
      ::drake::internal::ThrowWithValues(                                     \
          #condition, __func__, __FILE__, __LINE__                            \
          __VA_OPT__(, ACCUMULATE(__VA_ARGS__)));                             \
    }                                                                         \
  } while (0)
