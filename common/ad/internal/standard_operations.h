#pragma once

#include <cmath>
#include <iosfwd>

/* This file contains free function operators for Drake's AutoDiff type.

The functions provide not only arithmetic (+,-,*,/) and boolean comparison
(<,<=,>,>=,==,!=) but also argument-dependent lookup ("ADL") compatibility with
the standard library's mathematical functions (abs, etc.)

(See https://en.cppreference.com/w/cpp/language/adl for details about ADL.)

A few functions for Eigen::numext are also added to argument-dependent lookup.

Functions that cannot preserve gradients will return a primitive type (`bool`
or `double`) instead of an AutoDiff.

NOTE: This file should never be included directly, rather only from auto_diff.h
in a very specific order. */

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

/** ADL overload to mimic std::max from `<algorithm>`.
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

/** ADL overload to mimic std::max from `<algorithm>`.
When `a` and `b` are equal, retains the derivatives of `a`. */
inline AutoDiff max(AutoDiff a, double b) {
  if (a.value() < b) {
    a = b;
  }
  return a;
}

/** ADL overload to mimic std::max from `<algorithm>`.
When `a` and `b` are equal, retains the derivatives of `b`. */
inline AutoDiff max(double a, AutoDiff b) {
  if (a < b.value()) {
    return b;
  }
  b = a;
  return b;
}

/** ADL overload to mimic std::min from `<algorithm>`.
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

/** ADL overload to mimic std::min from `<algorithm>`.
When `a` and `b` are equal, retains the derivatives of `a`. */
inline AutoDiff min(AutoDiff a, double b) {
  if (b < a.value()) {
    a = b;
  }
  return a;
}

/** ADL overload to mimic std::min from `<algorithm>`.
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

/** ADL overload to mimic std::abs from `<cmath>`. */
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

/// @name Math functions: Exponential and Power functions
///
/// https://en.cppreference.com/w/cpp/numeric/math#Exponential_functions
///
/// https://en.cppreference.com/w/cpp/numeric/math#Power_functions
//@{

/** ADL overload to mimic std::exp from `<cmath>`. */
AutoDiff exp(AutoDiff x);

/** ADL overload to mimic std::log from `<cmath>`. */
AutoDiff log(AutoDiff x);

/** ADL overload to mimic std::pow from `<cmath>`.

The resulting partial derivative ∂/∂vᵢ is undefined (i.e., NaN) for all of the
following cases:
- ∂base/∂vᵢ is non-zero and either:
  - base, exp, or pow(base, exp) not finite, or
  - base is 0 and exp < 0
- ∂exp/∂vᵢ is non-zero and either:
  - base, exp, or pow(base, exp) not finite, or
  - base is < 0

In all other cases, if the base and exp partial derivatives were well-defined
then the resulting partial derivatives will also be well-defined. */
AutoDiff pow(AutoDiff base, const AutoDiff& exp);

/** ADL overload to mimic std::pow from `<cmath>`.
Refer to pow(AutoDiff,const AutoDiff&) for an explanation of special cases. */
AutoDiff pow(double base, const AutoDiff& exp);

/** ADL overload to mimic std::pow from `<cmath>`.
Refer to pow(AutoDiff,const AutoDiff&) for an explanation of special cases. */
AutoDiff pow(AutoDiff base, double exp);

/** ADL overload to mimic std::sqrt from `<cmath>`. */
AutoDiff sqrt(AutoDiff x);

//@}

/// @name Math functions: Trigonometric functions
///
/// https://en.cppreference.com/w/cpp/numeric/math#Trigonometric_functions
//@{

/** ADL overload to mimic std::sin from `<cmath>`. */
AutoDiff sin(AutoDiff x);

/** ADL overload to mimic std::cos from `<cmath>`. */
AutoDiff cos(AutoDiff x);

/** ADL overload to mimic std::tan from `<cmath>`. */
AutoDiff tan(AutoDiff x);

/** ADL overload to mimic std::asin from `<cmath>`. */
AutoDiff asin(AutoDiff x);

/** ADL overload to mimic std::acos from `<cmath>`. */
AutoDiff acos(AutoDiff x);

/** ADL overload to mimic std::atan from `<cmath>`. */
AutoDiff atan(AutoDiff x);

/** ADL overload to mimic std::atan2 from `<cmath>`. */
AutoDiff atan2(AutoDiff a, const AutoDiff& b);

/** ADL overload to mimic std::atan2 from `<cmath>`. */
AutoDiff atan2(AutoDiff a, double b);

/** ADL overload to mimic std::atan2 from `<cmath>`. */
AutoDiff atan2(double a, AutoDiff b);

//@}

/// @name Math functions: Hyperbolic functions
///
/// https://en.cppreference.com/w/cpp/numeric/math#Hyperbolic_functions
//@{

/** ADL overload to mimic std::sinh from `<cmath>`. */
AutoDiff sinh(AutoDiff x);

/** ADL overload to mimic std::cosh from `<cmath>`. */
AutoDiff cosh(AutoDiff x);

/** ADL overload to mimic std::tanh from `<cmath>`. */
AutoDiff tanh(AutoDiff x);

//@}

/// @name Math functions: Nearest integer floating point operations
///
/// https://en.cppreference.com/w/cpp/numeric/math#Nearest_integer_floating_point_operations
///
/// https://en.cppreference.com/w/cpp/numeric/math#Floating_point_manipulation_functions
//@{

/** ADL overload to mimic std::ceil from `<cmath>`.
The result's derivatives are always zero. */
inline double ceil(const AutoDiff& x) {
  return std::ceil(x.value());
}

/** ADL overload to mimic std::floor from `<cmath>`.
The result's derivatives are always zero. */
inline double floor(const AutoDiff& x) {
  return std::floor(x.value());
}

/** ADL overload to mimic std::round from `<cmath>`.
The result's derivatives are always zero. */
inline double round(const AutoDiff& x) {
  return std::round(x.value());
}

/** ADL overload to mimic std::nexttoward from `<cmath>`.
The result's derivatives are always zero. */
inline double nexttoward(const AutoDiff& from, long double to) {
  return std::nexttoward(from.value(), to);
}

/** ADL overload to mimic std::copysign from `<cmath>`. */
AutoDiff copysign(const AutoDiff& mag, const AutoDiff& sgn);

/** ADL overload to mimic std::isfinite from `<cmath>`.
Because the return type is `bool`, the derivatives are not preserved. */
inline bool isfinite(const AutoDiff& x) {
  return std::isfinite(x.value());
}

/** ADL overload to mimic std::isinf from `<cmath>`.
Because the return type is `bool`, the derivatives are not preserved. */
inline bool isinf(const AutoDiff& x) {
  return std::isinf(x.value());
}

/** ADL overload to mimic std::isnan from `<cmath>`.
Because the return type is `bool`, the derivatives are not preserved. */
inline bool isnan(const AutoDiff& x) {
  return std::isnan(x.value());
}

//@}

}  // namespace ad
}  // namespace drake
