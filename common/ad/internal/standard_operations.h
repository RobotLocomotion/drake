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

/// @name Math functions: Trigonometric functions
///
/// https://en.cppreference.com/w/cpp/numeric/math#Trigonometric_functions
//@{

/** ADL overload to mimic std::sin from <cmath>. */
AutoDiff sin(AutoDiff x);

/** ADL overload to mimic std::cos from <cmath>. */
AutoDiff cos(AutoDiff x);

/** ADL overload to mimic std::tan from <cmath>. */
AutoDiff tan(AutoDiff x);

/** ADL overload to mimic std::asin from <cmath>. */
AutoDiff asin(AutoDiff x);

/** ADL overload to mimic std::acos from <cmath>. */
AutoDiff acos(AutoDiff x);

/** ADL overload to mimic std::atan from <cmath>. */
AutoDiff atan(AutoDiff x);

/** ADL overload to mimic std::atan2 from <cmath>. */
AutoDiff atan2(AutoDiff a, const AutoDiff& b);

/** ADL overload to mimic std::atan2 from <cmath>. */
AutoDiff atan2(AutoDiff a, double b);

/** ADL overload to mimic std::atan2 from <cmath>. */
AutoDiff atan2(double a, AutoDiff b);

//@}

/// @name Math functions: Hyperbolic functions
///
/// https://en.cppreference.com/w/cpp/numeric/math#Hyperbolic_functions
//@{

/** ADL overload to mimic std::sinh from <cmath>. */
AutoDiff sinh(AutoDiff x);

/** ADL overload to mimic std::cosh from <cmath>. */
AutoDiff cosh(AutoDiff x);

/** ADL overload to mimic std::tanh from <cmath>. */
AutoDiff tanh(AutoDiff x);

//@}

/// @name Miscellaneous functions
//@{

/** Outputs the `value()` part of x to the stream.
To output the derivatives use `<< x.derivatives().transpose()`. */
std::ostream& operator<<(std::ostream& s, const AutoDiff& x);

//@}

}  // namespace ad
}  // namespace drake
