#pragma once

#include <type_traits>

/// @file
/// Provides a convenient wrapper to throw an exception when a condition is
/// unmet.  This is similar to an assertion, but uses exceptions instead of
/// ::abort(), and cannot be disabled.

namespace drake {
namespace detail {
// Throw an error message.
void Throw(const char* condition, const char* func, const char* file, int line);
}  // namespace detail
}  // namespace drake

/// Evaluates @p condition and iff the value is false will throw an exception
/// with a message showing at least the condition text, function name, file,
/// and line.
#define DRAKE_THROW_UNLESS(condition)                                   \
  do {                                                                  \
    static_assert(                                                      \
        std::is_convertible<decltype(condition), bool>::value,          \
        "Throw condition should be bool-convertible.");                 \
    if (!(condition)) {                                                 \
      ::drake::detail::Throw(#condition, __func__, __FILE__, __LINE__); \
    }                                                                   \
  } while (0)
