#pragma once

#include <cmath>
#include <iosfwd>

/* This file contains free function operators for Drake's AutoDiff type.

The functions provide arithmetic (+,-,*,/) for now and more to come later.

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
  // Special case to avoid changing b.partials() via a.partials() aliasing.
  if (&a == &b) {
    // ∂/∂x a * a = 2aa'
    a.partials().Mul(2.0 * a.value());
  } else {
    a.partials().Mul(b.value());
    a.partials().AddScaled(a.value(), b.partials());
  }
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
  // Special case to avoid changing b.partials() via a.partials() aliasing.
  if (&a == &b) {
    // ∂/∂x a / a = 0
    a.partials().SetZero();
  } else {
    a.partials().Mul(b.value());
    a.partials().AddScaled(-a.value(), b.partials());
    a.partials().Div(b.value() * b.value());
  }
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

/// @name Miscellaneous functions
//@{

/** Outputs the `value()` part of x to the stream.
To output the derivatives use `<< x.derivatives().transpose()`. */
std::ostream& operator<<(std::ostream& s, const AutoDiff& x);

//@}

}  // namespace ad
}  // namespace drake
