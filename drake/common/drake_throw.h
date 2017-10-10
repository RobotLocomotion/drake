#pragma once

#include <string>
#include <type_traits>

#include "drake/common/drake_assert.h"

/// @file
/// Provides a convenient wrapper to throw an exception when a condition is
/// unmet.  This is similar to an assertion, but uses exceptions instead of
/// ::abort(), and cannot be disabled.

namespace drake {
namespace detail {
// Throw an error message.
__attribute__((noreturn)) /* gcc is ok with [[noreturn]]; clang is not. */
void Throw(const char* condition, const char* func, const char* file, int line);
__attribute__((noreturn)) /* gcc is ok with [[noreturn]]; clang is not. */
void ThrowCompare(
    const char* condition,
    const std::string& value1, const std::string& value2,
    const char* func, const char* file, int line);
}  // namespace detail
}  // namespace drake

/// Evaluates @p condition and iff the value is false will throw an exception
/// with a message showing at least the condition text, function name, file,
/// and line.
#define DRAKE_THROW_UNLESS(condition)                                        \
  do {                                                                       \
    typedef ::drake::assert::ConditionTraits<                                \
        typename std::remove_cv<decltype(condition)>::type> Trait;           \
    static_assert(Trait::is_valid, "Condition should be bool-convertible."); \
    if (!Trait::Evaluate(condition)) {                                       \
      ::drake::detail::Throw(#condition, __func__, __FILE__, __LINE__);      \
    }                                                                        \
  } while (0)

/// Evaluates `term1 == term2` and iff the value is false will throw an
/// exception with a message showing at least the condition text, function
/// name, file, and line.
#define DRAKE_THROW_UNLESS_EQ(term1, term2)                                  \
  do {                                                                       \
    const auto& drake_throw_unless_eq_term_1 = term1;                        \
    const auto& drake_throw_unless_eq_term_2 = term2;                        \
    typedef ::drake::assert::ConditionTraits<                                \
        typename std::remove_cv<decltype(term1 == term2)>::type> Trait;      \
    static_assert(Trait::is_valid, "Condition should be bool-convertible."); \
    if (!Trait::Evaluate(                                                    \
            drake_throw_unless_eq_term_1 ==                                  \
            drake_throw_unless_eq_term_2)) {                                 \
      ::drake::detail::ThrowCompare(                                         \
           #term1 " == " #term2,                                             \
           std::to_string(drake_throw_unless_eq_term_1),                     \
           std::to_string(drake_throw_unless_eq_term_2),                     \
           __func__, __FILE__, __LINE__);                                    \
    }                                                                        \
  } while (0)
