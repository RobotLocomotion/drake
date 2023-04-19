#pragma once

#include <string>
#include <type_traits>
#include <vector>

#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/eigen_types.h"
#include "drake/common/fmt_eigen.h"

/** @file
 Provides a convenient wrapper to throw an exception when a condition is
 unmet.  This is similar to an assertion, but uses exceptions instead of
 ::abort(), and cannot be disabled.
 */

namespace drake {
namespace internal {
// Throw an error message.
[[noreturn]] void Throw(const char* condition, const char* func,
                        const char* file, int line,
                        std::string_view suffix = std::string_view());

/* The following infrastructure lets us iterate through a list of macro variadic
 arguments (up to length four).
 See https://codecraft.co/2014/11/25/variadic-macros-tricks/ for explanation.
*/

// Support up to *four* variadic arguments.
#define _GET_NTH_ARG(_1, _2, _3, _4, N, ...) N
// Macros for encoding a value expression into its value. _e is short for
// _encode.
#define _e(s, x) ::drake::internal::RecordExpressionValue(#x, x, s);
#define _e_0(...)
#define _e_1(s, value) _e(s, value)
#define _e_2(s, value, ...) _e(s, value) _e_1(s, __VA_ARGS__)
#define _e_3(s, value, ...) _e(s, value) _e_2(s, __VA_ARGS__)
#define _e_4(s, value, ...) _e(s, value) _e_3(s, __VA_ARGS__)

#define ENCODE_EACH(s, ...)                                                   \
    /* NOLINTNEXTLINE(whitespace/comma) */                                    \
    _GET_NTH_ARG(__VA_ARGS__ __VA_OPT__(,) _e_4, _e_3, _e_2, _e_1, _e_0)      \
        (s, __VA_ARGS__)


// Records the spelling of an expression and its reported value as:
//
//   expr = value
//
// adding it to the vector of recorded expression values.
template <typename T>
void RecordExpressionValue(const char* expr, const T& value,
                           std::vector<std::string>* recorded) {
  std::string value_s;
  if constexpr (::drake::is_eigen_type<T>::value) {
    value_s = fmt::format("{}", fmt_eigen(value));
  } else {
    if constexpr (std::is_convertible_v<T, std::string>) {
      // Give string-like values surrounding quotes.
      value_s = fmt::format("\"{}\"", value);
    } else {
      value_s = fmt::format("{}", value);
    }
  }
  recorded->emplace_back(fmt::format("{} = {}", expr, value_s));
}

[[noreturn]] void ThrowWithSuffix(const std::string& condition,
                                  const char* func, const char* file, int line,
                                  const std::string& suffix);
}  // namespace internal
}  // namespace drake

/** Evaluates @p condition and iff the value is false will throw an exception
 with a message showing at least the condition text, function name, file,
 and line.

 In addition to the @p condition, up to four value expressions can be provided.
 Each value expression and its value will be included in the error message. For
 example:

    `DRAKE_THROW_UNLESS(v1.norm() == 1, v1, v1.norm());`

 Will include the the values of the vector v1 as well as its norm in the
 message. If too many value expressions are specified, this will most likely
 produce a compiler error referencing "ENCODE_EACH".

 The condition must not be a pointer, where we'd implicitly rely on its
 nullness. Instead, always write out "!= nullptr" to be precise.

 Correct: `DRAKE_THROW_UNLESS(foo != nullptr);`
 Incorrect: `DRAKE_THROW_UNLESS(foo);`

 Because this macro is intended to provide a useful exception message to
 users, we should err on the side of extra detail about the failure. The
 meaning of "foo" isolated within error message text does not make it
 clear that a null pointer is the proximate cause of the problem.
 */
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
      std::vector<std::string> expr_strings;                                  \
      ENCODE_EACH(&expr_strings, __VA_ARGS__);        \
      ::drake::internal::Throw(                                               \
          #condition, __func__, __FILE__, __LINE__,                           \
          fmt::format("{}{}", fmt::join(expr_strings, ", "),                  \
                      expr_strings.size() > 0 ? "." : ""));                   \
    }                                                                         \
  } while (0)
