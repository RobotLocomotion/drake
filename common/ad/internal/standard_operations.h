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
inline AutoDiff sin(AutoDiff x) {
  // ∂/∂x sin(x) = cos(x)
  x.partials().Mul(std::cos(x.value()));
  x.value() = std::sin(x.value());
  return x;
}

/** ADL overload to mimic std::cos from <cmath>. */
inline AutoDiff cos(AutoDiff x) {
  // ∂/∂x cos(x) = -sin(x)
  x.partials().Mul(-std::sin(x.value()));
  x.value() = std::cos(x.value());
  return x;
}

/** ADL overload to mimic std::tan from <cmath>. */
inline AutoDiff tan(AutoDiff x) {
  // ∂/∂x tan(x) = sec²(x) = cos⁻²(x)
  const double cos_x = std::cos(x.value());
  x.partials().Div(cos_x * cos_x);
  x.value() = std::tan(x.value());
  return x;
}

/** ADL overload to mimic std::asin from <cmath>. */
inline AutoDiff asin(AutoDiff x) {
  // ∂/∂x acos(x) = 1 / sqrt(1 - x²)
  const double x2 = x.value() * x.value();
  x.partials().Div(std::sqrt(1 - x2));
  x.value() = std::asin(x.value());
  return x;
}

/** ADL overload to mimic std::acos from <cmath>. */
inline AutoDiff acos(AutoDiff x) {
  // ∂/∂x acos(x) = -1 / sqrt(1 - x²)
  const double x2 = x.value() * x.value();
  x.partials().Div(-std::sqrt(1 - x2));
  x.value() = std::acos(x.value());
  return x;
}

/** ADL overload to mimic std::atan from <cmath>. */
inline AutoDiff atan(AutoDiff x) {
  // ∂/∂x atan(x) = 1 / (1 + x²)
  const double x2 = x.value() * x.value();
  x.partials().Div(1 + x2);
  x.value() = std::atan(x.value());
  return x;
}

/** ADL overload to mimic std::atan2 from <cmath>. */
inline AutoDiff atan2(AutoDiff a, const AutoDiff& b) {
  // ∂/∂x atan2(a, b) = (ba' - ab')/(a² + b²)
  const double norm = a.value() * a.value() + b.value() * b.value();
  a.partials().Mul(b.value());
  a.partials().AddScaled(-a.value(), b.partials());
  a.partials().Div(norm);
  a.value() = std::atan2(a.value(), b.value());
  return a;
}

/** ADL overload to mimic std::atan2 from <cmath>. */
inline AutoDiff atan2(AutoDiff a, double b) {
  // ∂/∂x atan2(a, b) = (ba' - ab')/(a² + b²) = ba'/(a² + b²)
  const double norm = a.value() * a.value() + b * b;
  a.partials().Mul(b / norm);
  a.value() = std::atan2(a.value(), b);
  return a;
}

/** ADL overload to mimic std::atan2 from <cmath>. */
inline AutoDiff atan2(double a, AutoDiff b) {
  // ∂/∂x atan2(a, b) = (ba' - ab')/(a² + b²) = -ab'/(a² + b²)
  const double norm = a * a + b.value() * b.value();
  b.partials().Mul(-a / norm);
  b.value() = std::atan2(a, b.value());
  return b;
}

//@}

/// @name Math functions: Hyperbolic functions
///
/// https://en.cppreference.com/w/cpp/numeric/math#Hyperbolic_functions
//@{

/** ADL overload to mimic std::sinh from <cmath>. */
inline AutoDiff sinh(AutoDiff x) {
  // ∂/∂x sinh(x) = cosh(x)
  x.partials().Mul(std::cosh(x.value()));
  x.value() = std::sinh(x.value());
  return x;
}

/** ADL overload to mimic std::cosh from <cmath>. */
inline AutoDiff cosh(AutoDiff x) {
  // ∂/∂x cosh(x) = sinh(x)
  x.partials().Mul(std::sinh(x.value()));
  x.value() = std::cosh(x.value());
  return x;
}

/** ADL overload to mimic std::tanh from <cmath>. */
inline AutoDiff tanh(AutoDiff x) {
  // ∂/∂x tanh(x) = 1 - tanh²(x)
  const double new_value = std::tanh(x.value());
  x.partials().Mul(1 - (new_value * new_value));
  x.value() = new_value;
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
