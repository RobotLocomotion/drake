#pragma once

#include <cmath>
#include <iosfwd>

/* This file contains free function operators for Drake's AutoDiff type.

The functions provide not only arithmetic (+,-,*,/) and boolean comparison
(<,<=,>,>=,==,!=) but also argument-dependent lookup ("ADL") compatiblity with
the standard library's mathematical functions (abs, sin, round, isfinite, etc.)

(See https://en.cppreference.com/w/cpp/language/adl for details about ADL.)

A few functions for Eigen::numext are also added to argument-dependent lookup.

Functions that cannot preserve gradients will return a primitive type (`bool`
or `double`) instead of an AutoDiff.

NOTE: This file should never be included directly, rather only from
auto_diff.h in a very specific order. */

namespace drake {
namespace autodiff {

// === Arithmetic functions ===

// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator+=(AutoDiff& a, const AutoDiff& b) {
  // ∂/∂x a + b = a' + b'
  a.partials().Add(b.partials());
  a.value() += b.value();
  return a;
}

// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator+=(AutoDiff& a, double b) {
  // ∂/∂x a + b = a' + b' == a'
  a.value() += b;
  return a;
}

// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator-=(AutoDiff& a, const AutoDiff& b) {
  // ∂/∂x a - b = a' - b'
  a.partials().AddScaled(-1, b.partials());
  a.value() -= b.value();
  return a;
}

// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator-=(AutoDiff& a, double b) {
  // ∂/∂x a - b = a' - b' == a'
  a.value() -= b;
  return a;
}

// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator*=(AutoDiff& a, const AutoDiff& b) {
  // ∂/∂x a * b = ba' + ab'
  a.partials().Mul(b.value());
  a.partials().AddScaled(a.value(), b.partials());
  a.value() *= b.value();
  return a;
}

// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator*=(AutoDiff& a, double b) {
  // ∂/∂x a * b = ba' + ab' = ba'
  a.partials().Mul(b);
  a.value() *= b;
  return a;
}

// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator/=(AutoDiff& a, const AutoDiff& b) {
  // ∂/∂x a / b = (ba' - ab') / b²
  a.partials().Mul(b.value());
  a.partials().AddScaled(-a.value(), b.partials());
  a.partials().Div(b.value() * b.value());
  a.value() /= b.value();
  return a;
}

// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator/=(AutoDiff& a, double b) {
  // ∂/∂x a / b = (ba' - ab') / b² = a'/b
  return a *= (1 / b);
}

inline AutoDiff operator+(AutoDiff a, const AutoDiff& b) {
  a += b;
  return a;
}

inline AutoDiff operator+(AutoDiff a, double b) {
  a += b;
  return a;
}

inline AutoDiff operator+(double a, AutoDiff b) {
  b += a;
  return b;
}

inline AutoDiff operator-(AutoDiff a, const AutoDiff& b) {
  a -= b;
  return a;
}

inline AutoDiff operator-(AutoDiff a, double b) {
  a -= b;
  return a;
}

inline AutoDiff operator-(double a, AutoDiff b) {
  b *= -1;
  b += a;
  return b;
}

inline AutoDiff operator-(AutoDiff x) {
  x *= -1;
  return x;
}

inline AutoDiff operator*(AutoDiff a, const AutoDiff& b) {
  a *= b;
  return a;
}

inline AutoDiff operator*(AutoDiff a, double b) {
  a *= b;
  return a;
}

inline AutoDiff operator*(double a, AutoDiff b) {
  b *= a;
  return b;
}

inline AutoDiff operator/(AutoDiff a, const AutoDiff& b) {
  a /= b;
  return a;
}

inline AutoDiff operator/(AutoDiff a, double b) {
  a /= b;
  return a;
}

inline AutoDiff operator/(double a, const AutoDiff& b) {
  AutoDiff result{a};
  result /= b;
  return result;
}

// === Comparison functions ===

inline bool operator<(const AutoDiff& a, const AutoDiff& b) {
  return a.value() < b.value();
}

inline bool operator<=(const AutoDiff& a, const AutoDiff& b) {
  return a.value() <= b.value();
}

inline bool operator>(const AutoDiff& a, const AutoDiff& b) {
  return a.value() > b.value();
}

inline bool operator>=(const AutoDiff& a, const AutoDiff& b) {
  return a.value() >= b.value();
}

inline bool operator==(const AutoDiff& a, const AutoDiff& b) {
  return a.value() == b.value();
}

inline bool operator!=(const AutoDiff& a, const AutoDiff& b) {
  return a.value() != b.value();
}

inline bool operator<(const AutoDiff& a, double b) {
  return a.value() < b;
}

inline bool operator<=(const AutoDiff& a, double b) {
  return a.value() <= b;
}

inline bool operator>(const AutoDiff& a, double b) {
  return a.value() > b;
}

inline bool operator>=(const AutoDiff& a, double b) {
  return a.value() >= b;
}

inline bool operator==(const AutoDiff& a, double b) {
  return a.value() == b;
}

inline bool operator!=(const AutoDiff& a, double b) {
  return a.value() != b;
}

inline bool operator<(double a, const AutoDiff& b) {
  return a < b.value();
}

inline bool operator<=(double a, const AutoDiff& b) {
  return a <= b.value();
}

inline bool operator>(double a, const AutoDiff& b) {
  return a > b.value();
}

inline bool operator>=(double a, const AutoDiff& b) {
  return a >= b.value();
}

inline bool operator==(double a, const AutoDiff& b) {
  return a == b.value();
}

inline bool operator!=(double a, const AutoDiff& b) {
  return a != b.value();
}

// === Math functions ===

/** ADL overload to mimic std::abs from <cmath>. */
inline AutoDiff abs(AutoDiff x) {
  // Conditionally negate negative numbers.
  if (x.value() < 0) {
    x *= -1;
  }
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

/** ADL overload to mimic std::asin from <cmath>. */
inline AutoDiff asin(AutoDiff x) {
  // ∂/∂x acos(x) = 1 / sqrt(1 - x²)
  const double x2 = x.value() * x.value();
  x.partials().Div(std::sqrt(1 - x2));
  x.value() = std::asin(x.value());
  return x;
}

/** ADL overload to mimic std::atan from <cmath>. */
inline AutoDiff atan(AutoDiff x) {
  // ∂/∂x atan(x) = 1 / sqrt(1 + x²)
  const double x2 = x.value() * x.value();
  x.partials().Div(1 + x2);
  x.value() = std::atan(x.value());
  return x;
}

/** ADL overload to mimic std::cos from <cmath>. */
inline AutoDiff cos(AutoDiff x) {
  // ∂/∂x cos(x) = -sin(x)
  x.partials().Mul(-std::sin(x.value()));
  x.value() = std::cos(x.value());
  return x;
}

/** ADL overload to mimic std::cosh from <cmath>. */
inline AutoDiff cosh(AutoDiff x) {
  // ∂/∂x cosh(x) = sinh(x)
  x.partials().Mul(std::sinh(x.value()));
  x.value() = std::cosh(x.value());
  return x;
}

/** ADL overload to mimic std::exp from <cmath>. */
inline AutoDiff exp(AutoDiff x) {
  // ∂/∂x eˣ = eˣ
  const double new_value = std::exp(x.value());
  x.partials().Mul(new_value);
  x.value() = new_value;
  return x;
}

/** ADL overload to mimic std::log from <cmath>. */
inline AutoDiff log(AutoDiff x) {
  // ∂/∂x ln(x) = x⁻¹
  x.partials().Div(x.value());
  x.value() = std::log(x.value());
  return x;
}

/** ADL overload to mimic std::sin from <cmath>. */
inline AutoDiff sin(AutoDiff x) {
  // ∂/∂x sin(x) = cos(x)
  x.partials().Mul(std::cos(x.value()));
  x.value() = std::sin(x.value());
  return x;
}

/** ADL overload to mimic std::sinh from <cmath>. */
inline AutoDiff sinh(AutoDiff x) {
  // ∂/∂x sinh(x) = cosh(x)
  x.partials().Mul(std::cosh(x.value()));
  x.value() = std::sinh(x.value());
  return x;
}

/** ADL overload to mimic std::sqrt from <cmath>. */
inline AutoDiff sqrt(AutoDiff x) {
  // ∂/∂x x¹ᐟ² = ½x⁻¹ᐟ² = 1/(2x¹ᐟ²)
  const double new_value = std::sqrt(x.value());
  x.partials().Div(2 * new_value);
  x.value() = new_value;
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

/** ADL overload to mimic std::tanh from <cmath>. */
inline AutoDiff tanh(AutoDiff x) {
  // ∂/∂x tanh(x) = 1 - tanh²(x) = cos⁻²(x)
  const double new_value = std::tanh(x.value());
  x.partials().Mul(1 - (new_value * new_value));
  x.value() = new_value;
  return x;
}

/** ADL overload to mimic std::atan2 from <cmath>. */
inline AutoDiff atan2(AutoDiff a, const AutoDiff& b) {
  const double norm = a.value() * a.value() + b.value() * b.value();
  a.partials().Mul(b.value());
  a.partials().AddScaled(-a.value(), b.partials());
  a.partials().Div(norm);
  a.value() = std::atan2(a.value(), b.value());
  return a;
}

/** ADL overload to mimic std::atan2 from <cmath>. */
inline AutoDiff atan2(AutoDiff a, double b) {
  const double norm = a.value() * a.value() + b * b;
  a.partials().Mul(b / norm);
  a.value() = std::atan2(a.value(), b);
  return a;
}

/** ADL overload to mimic std::atan2 from <cmath>. */
inline AutoDiff atan2(double a, AutoDiff b) {
  const double norm = a * a + b.value() * b.value();
  b.partials().Mul(-a / norm);
  b.value() = std::atan2(a, b.value());
  return b;
}

/** ADL overload to mimic std::max from <algorithm>.
When `a` and `b` are equal, retains the partials of `a`. */
inline AutoDiff max(AutoDiff a, const AutoDiff& b) {
  if (b.value() > a.value()) {
    a = b;
  }
  return a;
}

/** ADL overload to mimic std::max from <algorithm>.
When `a` and `b` are equal, retains the partials of `a`. */
inline AutoDiff max(AutoDiff a, double b) {
  if (b > a.value()) {
    a = b;
  }
  return a;
}

/** ADL overload to mimic std::max from <algorithm>.
When `a` and `b` are equal, retains the partials of `b`. */
inline AutoDiff max(double a, AutoDiff b) {
  if (a > b.value()) {
    b = a;
  }
  return b;
}

// TODO(jwnimmer-tri) std::min and std::max return by-const-reference;
// we should probably do the same.  If they are equal, we should also
// take whichever one has non-empty gradients.

/** ADL overload to mimic std::min from <algorithm>.
When `a` and `b` are equal, retains the partials of `a`. */
inline AutoDiff min(AutoDiff a, const AutoDiff& b) {
  if (b.value() < a.value()) {
    a = b;
  }
  return a;
}

/** ADL overload to mimic std::min from <algorithm>.
When `a` and `b` are equal, retains the partials of `a`. */
inline AutoDiff min(AutoDiff a, double b) {
  if (b < a.value()) {
    a = b;
  }
  return a;
}

/** ADL overload to mimic std::min from <algorithm>.
When `a` and `b` are equal, retains the partials of `b`. */
// NOLINTNEXTLINE(build/include_what_you_use) false positive.
inline AutoDiff min(double a, AutoDiff b) {
  if (a < b.value()) {
    b = a;
  }
  return b;
}

/** ADL overload to mimic std::pow from <cmath>. */
inline AutoDiff pow(AutoDiff a, const AutoDiff& b) {
  // ∂/∂x aᵇ = aᵇ⁻¹ (b a' + a ln(a) b')
  const double a_bn1 = std::pow(a.value(), b.value() - 1);
  const double a_scale = a_bn1 * b.value();
  // TODO(jwnimmer-tri) Deal with a < 0 somehow.
  const double b_scale = a_bn1 * a.value() * std::log(a.value());
  a.partials().Mul(a_scale);
  a.partials().AddScaled(b_scale, b.partials());
  a.value() *= a_bn1;  // aaᵇ⁻¹ = aᵇ
  return a;
}

/** ADL overload to mimic std::pow from <cmath>. */
inline AutoDiff pow(double a, AutoDiff b) {
  // ∂/∂x aᵇ = aᵇ⁻¹ (b a' + a ln(a) b') but with a' == 0 we have
  // ∂/∂x aᵇ = aᵇ ln(a) b'
  const double new_value = std::pow(a, b.value());
  // TODO(jwnimmer-tri) Deal with a < 0 somehow.
  const double b_scale = new_value * std::log(a);
  b.partials().Mul(b_scale);
  b.value() = new_value;
  return a;
}

/** ADL overload to mimic std::pow from <cmath>. */
inline AutoDiff pow(AutoDiff a, double b) {
  // ∂/∂x aᵇ = aᵇ⁻¹ (b a' + a ln(a) b') but with b' == 0 we have
  // ∂/∂x aᵇ = aᵇ⁻¹ b a'
  const double a_bn1 = std::pow(a.value(), b - 1);
  const double a_scale = a_bn1 * b;
  a.partials().Mul(a_scale);
  a.value() *= a_bn1;  // aaᵇ⁻¹ = aᵇ
  return a;
}

/** ADL overload to mimic std::ceil from <cmath>.
Discards the derivatives. */
inline double ceil(const AutoDiff& x) {
  return std::ceil(x.value());
}

/** ADL overload to mimic std::floor from <cmath>.
Discards the derivatives. */
inline double floor(const AutoDiff& x) {
  return std::floor(x.value());
}

/** ADL overload to mimic std::nexttoward from <cmath>.
Discards the derivatives.*/
inline double nexttoward(const AutoDiff& from, long double to) {
  return std::nexttoward(from.value(), to);
}

/** ADL overload to mimic std::round from <cmath>.
Discards the derivatives. */
inline double round(const AutoDiff& x) {
  return std::round(x.value());
}

/** ADL overload to mimic std::isfinite from <cmath>. */
inline bool isfinite(const AutoDiff& x) {
  return std::isfinite(x.value());
}

/** ADL overload to mimic std::isinf from <cmath>. */
inline bool isinf(const AutoDiff& x) {
  return std::isinf(x.value());
}

/** ADL overload to mimic std::isnan from <cmath>. */
inline bool isnan(const AutoDiff& x) {
  return std::isnan(x.value());
}

// === Eigen::numext functions ===

/** ADL overload to mimic Eigen::numext::abs2. */
inline AutoDiff abs2(AutoDiff x) {
  // ∂/∂x x² = 2x
  x.partials().Mul(2 * x.value());
  x.value() *= x.value();
  return x;
}

// === Miscellaneous functions ===

/** Outputs the value() part of x to the stream. */
std::ostream& operator<<(std::ostream& s, const AutoDiff& x);

}  // namespace autodiff
}  // namespace drake
