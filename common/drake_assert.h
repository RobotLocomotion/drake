#pragma once

#include <type_traits>

/// @file
/// Provides Drake's assertion implementation.  This is intended to be used
/// both within Drake and by other software.  Drake's asserts can be armed
/// and disarmed independently from the system-wide asserts.

#ifdef DRAKE_DOXYGEN_CXX
/// @p DRAKE_ASSERT(condition) is similar to the built-in @p assert(condition)
/// from the C++ system header @p <cassert>.  Unless Drake's assertions are
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
#else  //  DRAKE_DOXYGEN_CXX

// Users should NOT set these; only this header should set them.
#ifdef DRAKE_ASSERT_IS_ARMED
# error Unexpected DRAKE_ASSERT_IS_ARMED defined.
#endif
#ifdef DRAKE_ASSERT_IS_DISARMED
# error Unexpected DRAKE_ASSERT_IS_DISARMED defined.
#endif

// Decide whether Drake assertions are enabled.
#if defined(DRAKE_ENABLE_ASSERTS) && defined(DRAKE_DISABLE_ASSERTS)
# error Conflicting assertion toggles.
#elif defined(DRAKE_ENABLE_ASSERTS)
# define DRAKE_ASSERT_IS_ARMED
#elif defined(DRAKE_DISABLE_ASSERTS) || defined(NDEBUG)
# define DRAKE_ASSERT_IS_DISARMED
#else
# define DRAKE_ASSERT_IS_ARMED
#endif

namespace drake {
namespace internal {
// Abort the program with an error message.
__attribute__((noreturn)) /* gcc is ok with [[noreturn]]; clang is not. */
void Abort(const char* condition, const char* func, const char* file, int line);
// Report an assertion failure; will either Abort(...) or throw.
__attribute__((noreturn)) /* gcc is ok with [[noreturn]]; clang is not. */
void AssertionFailed(
    const char* condition, const char* func, const char* file, int line);
}  // namespace internal
namespace assert {
// Allows for specialization of how to bool-convert Conditions used in
// assertions, in case they are not intrinsically convertible.  See
// symbolic_formula.h for an example use.  This is a public interface to
// extend; it is intended to be specialized by unusual Scalar types that
// require special handling.
template <typename Condition>
struct ConditionTraits {
  static constexpr bool is_valid = std::is_convertible<Condition, bool>::value;
  static bool Evaluate(const Condition& value) {
    return value;
  }
};
}  // namespace assert
}  // namespace drake

#define DRAKE_UNREACHABLE()                                             \
  ::drake::internal::Abort(                                             \
      "Unreachable code was reached?!", __func__, __FILE__, __LINE__)

#define DRAKE_DEMAND(condition)                                              \
  do {                                                                       \
    typedef ::drake::assert::ConditionTraits<                                \
        typename std::remove_cv<decltype(condition)>::type> Trait;           \
    static_assert(Trait::is_valid, "Condition should be bool-convertible."); \
    if (!Trait::Evaluate(condition)) {                                       \
      ::drake::internal::AssertionFailed(                                    \
           #condition, __func__, __FILE__, __LINE__);                        \
    }                                                                        \
  } while (0)

#ifdef DRAKE_ASSERT_IS_ARMED
// Assertions are enabled.
namespace drake {
constexpr bool kDrakeAssertIsArmed = true;
constexpr bool kDrakeAssertIsDisarmed = false;
}  // namespace drake
# define DRAKE_ASSERT(condition) DRAKE_DEMAND(condition)
# define DRAKE_ASSERT_VOID(expression) do {                     \
    static_assert(                                              \
        std::is_convertible<decltype(expression), void>::value, \
        "Expression should be void.");                          \
    expression;                                                 \
  } while (0)
#else
// Assertions are disabled, so just typecheck the expression.
namespace drake {
constexpr bool kDrakeAssertIsArmed = false;
constexpr bool kDrakeAssertIsDisarmed = true;
}  // namespace drake
# define DRAKE_ASSERT(condition) static_assert(                        \
    ::drake::assert::ConditionTraits<                                  \
        typename std::remove_cv<decltype(condition)>::type>::is_valid, \
    "Condition should be bool-convertible.");
# define DRAKE_ASSERT_VOID(expression) static_assert(           \
    std::is_convertible<decltype(expression), void>::value,     \
    "Expression should be void.")
#endif

#endif  // DRAKE_DOXYGEN_CXX
