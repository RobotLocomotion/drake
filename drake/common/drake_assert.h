#pragma once

#include <cassert>
#include <type_traits>

/// @file
/// Provides Drake's assertion implementation.  This is intended to be
/// used both within Drake and by other software.

#ifdef DRAKE_DOXYGEN_CXX
/// @p DRAKE_ASSERT(condition) is similar to the built-in @p assert(condition)
/// from the C++ system header @p <cassert>.  Unless Drake's assertions are
/// disarmed by the pre-processor definitions listed below, @p DRAKE_ASSERT
/// will evaluate @p condition and iff the value is false will ::abort() the
/// program with a message showing at least the condition text, file, and line.
///
/// Assertions are enabled or disabled using the following pre-processor macros:
/// - If @p DRAKE_ENABLE_ASSERTS is defined, then @p DRAKE_ASSERT is armed.
/// - If @p DRAKE_DISABLE_ASSERTS is defined, then @p DRAKE_ASSERT is disarmed.
/// - If both macros are defined, it is a compile-time error.
/// - If neither are defined, then NDEBUG governs assertions as usual.
///
/// One difference versus the standard @p assert(condition) is that the
/// @p condition within @p DRAKE_ASSERT is always syntax-checked, even if
/// Drake's assertions are disarmed.
///
/// Treat @p DRAKE_ASSERT like a statement -- it must always be used
/// in block scope, and must always be followed by a semicolon.
#define DRAKE_ASSERT(condition)
/// Like @p DRAKE_ASSERT, except is always armed.
#define DRAKE_ABORT_UNLESS(condition)
/// Like @p DRAKE_ABORT_UNLESS(false).
#define DRAKE_ABORT()
#else  //  DRAKE_DOXYGEN_CXX

// Users should NOT set this; only this header should set it.
#ifdef DRAKE_ASSERT_IS_VOID
# error Unexpected DRAKE_ASSERT_IS_VOID defined.
#endif

// Decide whether Drake assertions are enabled.
#if defined(DRAKE_ENABLE_ASSERTS) && defined(DRAKE_DISABLE_ASSERTS)
# error Conflicting assertion toggles.
#elif defined(DRAKE_ENABLE_ASSERTS)
#elif defined(DRAKE_DISABLE_ASSERTS) || defined(NDEBUG)
# define DRAKE_ASSERT_IS_VOID
#endif

namespace drake {
/// Abort the program with an error message.
void Abort(const char* condition, const char* func, const char* file, int line);
}

#define DRAKE_ABORT()                                           \
  ::drake::Abort(nullptr, __func__, __FILE__, __LINE__)

#define DRAKE_ABORT_UNLESS(condition)                           \
  do {                                                          \
    if (!(condition)) {                                         \
      ::drake::Abort(#condition, __func__, __FILE__, __LINE__); \
    }                                                           \
  } while (0)

#ifdef DRAKE_ASSERT_IS_VOID
// Assertions are disabled, so just typecheck the expression.
#define DRAKE_ASSERT(condition) static_assert(                  \
      std::is_convertible<decltype(condition), bool>::value,    \
      "Assertion condition should be bool-convertible.")
#else
// Assertions are enabled.
#define DRAKE_ASSERT(condition) DRAKE_ABORT_UNLESS(condition)
#endif

#endif  // DRAKE_DOXYGEN_CXX
