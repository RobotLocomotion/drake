/// @file
/// Provides necessary operations on double to have it as a ScalarType in drake.

#pragma once

namespace drake {
/// Provides if-then-else expression for double. The value returned by the
/// if-then-else expression is @p v_then if @p f_cond is @c true. Otherwise, it
/// returns @p v_else.

/// The semantics is similar but not exactly the same as C++'s conditional
/// expression constructed by its ternary operator, @c ?:. In
/// <tt>if_then_else(f_cond, v_then, v_else)</tt>, both of @p v_then and @p
/// v_else are evaluated regardless of the evaluation of @p f_cond. In contrast,
/// only one of @p v_then or @p v_else is evaluated in C++'s conditional
/// expression <tt>f_cond ? v_then : v_else</tt>.
inline double if_then_else(bool f_cond, double v_then, double v_else) {
  return f_cond ? v_then : v_else;
}

}  // namespace drake
