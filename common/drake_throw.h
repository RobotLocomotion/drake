#pragma once

#include <array>
#include <string>
#include <type_traits>
#include <utility>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/fmt.h"

/// @file
/// Provides a convenient wrapper to throw an exception when a condition is
/// unmet.  This is similar to an assertion, but uses exceptions instead of
/// `::abort()`, and cannot be disabled.

// TODO(SeanCurtis-TRI) We should merge the contents of this file into
// `drake_assert.h`. This file would simply include that file for backwards
// compatibility.

namespace drake {
namespace internal {

/* StringifyValue converts `value` into a string. Some types are given special
 treatment to ensure a specific format. */
template <typename T>
std::string StringifyValue(const T& value) {
  // TODO(SeanCurtis-TRI) This version only supports floats. As we seek to pass
  // *other* types (strings, paths, eigen types, etc.), we'll need to extend
  // the supported types here.
  if constexpr (std::is_floating_point_v<T>) {
    return fmt_floating_point(value);
  } else {
    // TODO(SeanCurtis-TRI): Under Noble, our compiler only includes this static
    // assert if the previous static conditions don't apply. So, we could simply
    // assert false here knowing that the user will get this compilation error
    // only for unsupported types.
    //
    // However, under Jammy, the static_assert is included even if this else
    // block doesn't apply for a particular T. Therefore, we can't rely on the
    // if constexpr structure to guard the assertion and have to rely on an
    // explicit type-derived condition. Once we drop Jammy, we can replace the
    // condition with `false`.
    //
    // However, while we still have Jammy, we need to make sure the assertion
    // condition evolves explicitly to cover any types we support in the future.
    static_assert(std::is_floating_point_v<T>,
                  "You've passed DRAKE_THROW_UNLESS an unsupported value "
                  "expression type. Please see issue #23468 for instructions "
                  "on how to request support for this value type.");
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
  [&]<size_t... I>(std::integer_sequence<size_t, I...>&&) {
    (((buffer.values[I].first = std::get<2 * I>(std::move(pairs_tuple)),
       buffer.values[I].second =
           StringifyValue(std::get<2 * I + 1>(std::move(pairs_tuple)))),
      ...));
  }(std::make_index_sequence<N / 2>{});

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

The current version only supports float-valued value expressions. Attempting to
provide a value expression of an unsupported type will result in a static_assert
failure. We will extend the set of supported types as needed. See Drake issue
#23468 for instructions on how to request support for new types. */
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
