#pragma once

#include <cmath>
#include <iosfwd>

/* This file contains free function operators for Drake's AutoDiff type.

The functions provide not only arithmetic (+,-,*,/) and boolean comparison
(<,<=,>,>=,==,!=) but also argument-dependent lookup ("ADL") compatiblity with
the standard library's mathematical functions (abs, etc.)

(See https://en.cppreference.com/w/cpp/language/adl for details about ADL.)

A few functions for Eigen::numext are also added to argument-dependent lookup.

Functions that cannot preserve gradients will return a primitive type (`bool`
or `double`) instead of an AutoDiff.

NOTE: This file should never be included directly, rather only from
auto_diff.h in a very specific order. */

namespace drake {
namespace ad {

/// @name Increment and decrement
///
/// https://en.cppreference.com/w/cpp/language/operators#Increment_and_decrement
//@{

/** Standard prefix increment operator (i.e., `++x`). */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator++(AutoDiff& x) {
  ++x.value();
  return x;
}

/** Standard postfix increment operator (i.e., `x++`). */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff operator++(AutoDiff& x, int) {
  AutoDiff result = x;
  ++x.value();
  return result;
}

/** Standard prefix decrement operator (i.e., `--x`). */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator--(AutoDiff& x) {
  --x.value();
  return x;
}

/** Standard postfix decrement operator (i.e., `x--`). */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff operator--(AutoDiff& x, int) {
  AutoDiff result = x;
  --x.value();
  return result;
}

//@}

/// @name Arithmetic operators
///
/// https://en.cppreference.com/w/cpp/language/operators#Binary_arithmetic_operators
//@{

/** Standard compound addition and assignment operator. */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator+=(AutoDiff& a, const AutoDiff& b) {
  // ∂/∂x a + b = a' + b'
  a.partials().Add(b.partials());
  a.value() += b.value();
  return a;
}

/** Standard compound addition and assignment operator. */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator+=(AutoDiff& a, double b) {
  // ∂/∂x a + b = a' + b' == a'
  a.value() += b;
  return a;
}

/** Standard compound subtraction and assignment operator. */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator-=(AutoDiff& a, const AutoDiff& b) {
  // ∂/∂x a - b = a' - b'
  a.partials().AddScaled(-1, b.partials());
  a.value() -= b.value();
  return a;
}

/** Standard compound subtraction and assignment operator. */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator-=(AutoDiff& a, double b) {
  // ∂/∂x a - b = a' - b' == a'
  a.value() -= b;
  return a;
}

/** Standard compound multiplication and assignment operator. */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator*=(AutoDiff& a, const AutoDiff& b) {
  // ∂/∂x a * b = ba' + ab'
  a.partials().Mul(b.value());
  a.partials().AddScaled(a.value(), b.partials());
  a.value() *= b.value();
  return a;
}

/** Standard compound multiplication and assignment operator. */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator*=(AutoDiff& a, double b) {
  // ∂/∂x a * b = ba' + ab' = ba'
  a.partials().Mul(b);
  a.value() *= b;
  return a;
}

/** Standard compound division and assignment operator. */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator/=(AutoDiff& a, const AutoDiff& b) {
  // ∂/∂x a / b = (ba' - ab') / b²
  a.partials().Mul(b.value());
  a.partials().AddScaled(-a.value(), b.partials());
  a.partials().Div(b.value() * b.value());
  a.value() /= b.value();
  return a;
}

/** Standard compound division and assignment operator. */
// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator/=(AutoDiff& a, double b) {
  // ∂/∂x a / b = (ba' - ab') / b² = a'/b
  a.partials().Div(b);
  a.value() /= b;
  return a;
}

/** Standard addition operator. */
inline AutoDiff operator+(AutoDiff a, const AutoDiff& b) {
  a += b;
  return a;
}

/** Standard addition operator. */
inline AutoDiff operator+(AutoDiff a, double b) {
  a += b;
  return a;
}

/** Standard addition operator. */
inline AutoDiff operator+(double a, AutoDiff b) {
  b += a;
  return b;
}

/** Standard unary plus operator. */
inline AutoDiff operator+(AutoDiff x) {
  return x;
}

/** Standard subtraction operator. */
inline AutoDiff operator-(AutoDiff a, const AutoDiff& b) {
  a -= b;
  return a;
}

/** Standard subtraction operator. */
inline AutoDiff operator-(AutoDiff a, double b) {
  a -= b;
  return a;
}

/** Standard subtraction operator. */
inline AutoDiff operator-(double a, AutoDiff b) {
  b *= -1;
  b += a;
  return b;
}

/** Standard unary minus operator. */
inline AutoDiff operator-(AutoDiff x) {
  x *= -1;
  return x;
}

/** Standard multiplication operator. */
inline AutoDiff operator*(AutoDiff a, const AutoDiff& b) {
  a *= b;
  return a;
}

/** Standard multiplication operator. */
inline AutoDiff operator*(AutoDiff a, double b) {
  a *= b;
  return a;
}

/** Standard multiplication operator. */
inline AutoDiff operator*(double a, AutoDiff b) {
  b *= a;
  return b;
}

/** Standard division operator. */
inline AutoDiff operator/(AutoDiff a, const AutoDiff& b) {
  a /= b;
  return a;
}

/** Standard division operator. */
inline AutoDiff operator/(AutoDiff a, double b) {
  a /= b;
  return a;
}

/** Standard division operator. */
inline AutoDiff operator/(double a, const AutoDiff& b) {
  AutoDiff result{a};
  result /= b;
  return result;
}

//@}

/// @name Comparison operators
///
/// https://en.cppreference.com/w/cpp/language/operators#Comparison_operators
//@{

/** Standard comparison operator. Discards the derivatives. */
inline bool operator<(const AutoDiff& a, const AutoDiff& b) {
  return a.value() < b.value();
}

/** Standard comparison operator. Discards the derivatives. */
inline bool operator<=(const AutoDiff& a, const AutoDiff& b) {
  return a.value() <= b.value();
}

/** Standard comparison operator. Discards the derivatives. */
inline bool operator>(const AutoDiff& a, const AutoDiff& b) {
  return a.value() > b.value();
}

/** Standard comparison operator. Discards the derivatives. */
inline bool operator>=(const AutoDiff& a, const AutoDiff& b) {
  return a.value() >= b.value();
}

/** Standard comparison operator. Discards the derivatives. */
inline bool operator==(const AutoDiff& a, const AutoDiff& b) {
  return a.value() == b.value();
}

/** Standard comparison operator. Discards the derivatives. */
inline bool operator!=(const AutoDiff& a, const AutoDiff& b) {
  return a.value() != b.value();
}

/** Standard comparison operator. Discards the derivatives. */
inline bool operator<(const AutoDiff& a, double b) {
  return a.value() < b;
}

/** Standard comparison operator. Discards the derivatives. */
inline bool operator<=(const AutoDiff& a, double b) {
  return a.value() <= b;
}

/** Standard comparison operator. Discards the derivatives. */
inline bool operator>(const AutoDiff& a, double b) {
  return a.value() > b;
}

/** Standard comparison operator. Discards the derivatives. */
inline bool operator>=(const AutoDiff& a, double b) {
  return a.value() >= b;
}

/** Standard comparison operator. Discards the derivatives. */
inline bool operator==(const AutoDiff& a, double b) {
  return a.value() == b;
}

/** Standard comparison operator. Discards the derivatives. */
inline bool operator!=(const AutoDiff& a, double b) {
  return a.value() != b;
}

/** Standard comparison operator. Discards the derivatives. */
inline bool operator<(double a, const AutoDiff& b) {
  return a < b.value();
}

/** Standard comparison operator. Discards the derivatives. */
inline bool operator<=(double a, const AutoDiff& b) {
  return a <= b.value();
}

/** Standard comparison operator. Discards the derivatives. */
inline bool operator>(double a, const AutoDiff& b) {
  return a > b.value();
}

/** Standard comparison operator. Discards the derivatives. */
inline bool operator>=(double a, const AutoDiff& b) {
  return a >= b.value();
}

/** Standard comparison operator. Discards the derivatives. */
inline bool operator==(double a, const AutoDiff& b) {
  return a == b.value();
}

/** Standard comparison operator. Discards the derivatives. */
inline bool operator!=(double a, const AutoDiff& b) {
  return a != b.value();
}

//@}

/// @name Minimum/maximum operations
///
/// https://en.cppreference.com/w/cpp/algorithm#Minimum.2Fmaximum_operations
//@{

/** ADL overload to mimic std::max from <algorithm>.
Note that like std::max, this function returns a reference to whichever
argument was chosen; it does not make a copy. When `a` and `b` are equal,
retains the derivatives of `a` (by returning `a`) unless `a` has empty
derivatives, in which case `b` is returned. */
inline const AutoDiff& max(const AutoDiff& a, const AutoDiff& b) {
  if (a.value() == b.value()) {
    return a.derivatives().size() > 0 ? a : b;
  }
  return a.value() < b.value() ? b : a;
}

/** ADL overload to mimic std::max from <algorithm>.
When `a` and `b` are equal, retains the derivatives of `a`. */
inline AutoDiff max(AutoDiff a, double b) {
  if (a.value() < b) {
    a = b;
  }
  return a;
}

/** ADL overload to mimic std::max from <algorithm>.
When `a` and `b` are equal, retains the derivatives of `b`. */
inline AutoDiff max(double a, AutoDiff b) {
  if (a < b.value()) {
    return b;
  }
  b = a;
  return b;
}

/** ADL overload to mimic std::min from <algorithm>.
Note that like std::min, this function returns a reference to whichever
argument was chosen; it does not make a copy. When `a` and `b` are equal,
retains the derivatives of `a` (by returning `a`) unless `a` has empty
derivatives, in which case `b` is returned. */
inline const AutoDiff& min(const AutoDiff& a, const AutoDiff& b) {
  if (a.value() == b.value()) {
    return a.derivatives().size() > 0 ? a : b;
  }
  return b.value() < a.value() ? b : a;
}

/** ADL overload to mimic std::min from <algorithm>.
When `a` and `b` are equal, retains the derivatives of `a`. */
inline AutoDiff min(AutoDiff a, double b) {
  if (b < a.value()) {
    a = b;
  }
  return a;
}

/** ADL overload to mimic std::min from <algorithm>.
When `a` and `b` are equal, retains the derivatives of `b`. */
// NOLINTNEXTLINE(build/include_what_you_use) false positive.
inline AutoDiff min(double a, AutoDiff b) {
  if (a < b.value()) {
    b = a;
  }
  return b;
}

//@}

/// @name Math functions: Basic operations
///
/// https://en.cppreference.com/w/cpp/numeric/math#Basic_operations
//@{

/** ADL overload to mimic std::abs from <cmath>. */
inline AutoDiff abs(AutoDiff x) {
  // Conditionally negate negative numbers.
  if (x.value() < 0) {
    x *= -1;
  }
  return x;
}

/** ADL overload to mimic Eigen::numext::abs2. */
inline AutoDiff abs2(AutoDiff x) {
  // ∂/∂x x² = 2x
  x.partials().Mul(2 * x.value());
  x.value() *= x.value();
  return x;
}

//@}

/// @name Miscellaneous functions
//@{

/** Outputs the `value()` part of x to the stream.
To output the derivatives use `<< x.derivatives().transpose()`. */
std::ostream& operator<<(std::ostream& s, const AutoDiff& x);

//@}

}  // namespace ad
}  // namespace drake
