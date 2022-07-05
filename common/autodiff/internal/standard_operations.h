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
  a.partials().Add(b.partials());
  a.value() += b.value();
  return a;
}

// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator+=(AutoDiff& a, double b) {
  a.value() += b;
  return a;
}

// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator-=(AutoDiff& a, const AutoDiff& b) {
  a.partials().AddScaled(-1.0, b.partials());
  a.value() -= b.value();
  return a;
}

// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator-=(AutoDiff& a, double b) {
  a.value() -= b;
  return a;
}

// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator*=(AutoDiff& a, const AutoDiff& b) {
  a.partials().Mul(b.value());
  a.partials().AddScaled(a.value(), b.partials());
  a.value() *= b.value();
  return a;
}

// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator*=(AutoDiff& a, double b) {
  a.partials().Mul(b);
  a.value() *= b;
  return a;
}

// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator/=(AutoDiff& a, const AutoDiff& b) {
  a.partials().Mul(b.value());
  a.partials().AddScaled(-a.value(), b.partials());
  a.partials().Mul(1.0 / (b.value() * b.value()));
  a.value() /= b.value();
  return a;
}

// NOLINTNEXTLINE(runtime/references) to match the required signature.
inline AutoDiff& operator/=(AutoDiff& a, double b) {
  return a *= (1.0 / b);
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
  b *= -1.0;
  b += a;
  return b;
}

inline AutoDiff operator-(AutoDiff x) {
  x *= -1.0;
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
  if (x.value() < 0.0) {
    x *= -1.0;
  }
  return x;
}

/** ADL overload to mimic std::acos from <cmath>. */
inline AutoDiff acos(AutoDiff x) {
  const double x2 = x.value() * x.value();
  x.partials().Mul(-1 / std::sqrt(1 - x2));
  x.value() = std::acos(x.value());
  return x;
}

/** ADL overload to mimic std::asin from <cmath>. */
inline AutoDiff asin(AutoDiff x) {
  const double x2 = x.value() * x.value();
  x.partials().Mul(1 / std::sqrt(1 - x2));
  x.value() = std::asin(x.value());
  return x;
}

/** ADL overload to mimic std::atan from <cmath>. */
inline AutoDiff atan(AutoDiff x) {
  const double x2 = x.value() * x.value();
  x.partials().Mul(1 / (1 + x2));
  x.value() = std::atan(x.value());
  return x;
}

/** ADL overload to mimic std::cos from <cmath>. */
inline AutoDiff cos(AutoDiff x) {
  x.partials().Mul(-std::sin(x.value()));
  x.value() = std::cos(x.value());
  return x;
}

/** ADL overload to mimic std::cosh from <cmath>. */
inline AutoDiff cosh(AutoDiff x) {
  x.partials().Mul(std::sinh(x.value()));
  x.value() = std::cosh(x.value());
  return x;
}

/** ADL overload to mimic std::exp from <cmath>. */
inline AutoDiff exp(AutoDiff x) {
  const double new_value = std::exp(x.value());
  x.partials().Mul(new_value);
  x.value() = new_value;
  return x;
}

/** ADL overload to mimic std::log from <cmath>. */
inline AutoDiff log(AutoDiff x) {
  x.partials().Mul(1 / x.value());
  x.value() = std::log(x.value());
  return x;
}

/** ADL overload to mimic std::sin from <cmath>. */
inline AutoDiff sin(AutoDiff x) {
  x.partials().Mul(std::cos(x.value()));
  x.value() = std::sin(x.value());
  return x;
}

/** ADL overload to mimic std::sinh from <cmath>. */
inline AutoDiff sinh(AutoDiff x) {
  x.partials().Mul(std::cosh(x.value()));
  x.value() = std::sinh(x.value());
  return x;
}

/** ADL overload to mimic std::sqrt from <cmath>. */
inline AutoDiff sqrt(AutoDiff x) {
  const double new_value = std::sqrt(x.value());
  x.partials().Mul(0.5 / new_value);
  x.value() = new_value;
  return x;
}

/** ADL overload to mimic std::tan from <cmath>. */
inline AutoDiff tan(AutoDiff x) {
  const double cos_x = std::cos(x.value());
  x.partials().Mul(1 / (cos_x * cos_x));
  x.value() = std::tan(x.value());
  return x;
}

/** ADL overload to mimic std::tanh from <cmath>. */
inline AutoDiff tanh(AutoDiff x) {
  const double cosh_x = std::cosh(x.value());
  x.partials().Mul(1 / (cosh_x * cosh_x));
  x.value() = std::tanh(x.value());
  return x;
}

/** ADL overload to mimic std::atan2 from <cmath>. */
inline AutoDiff atan2(AutoDiff a, const AutoDiff& b) {
  const double norm = a.value() * a.value() + b.value() * b.value();
  a.partials().Mul(b.value());
  a.partials().AddScaled(-a.value(), b.partials());
  a.partials().Mul(1.0 / norm);
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

inline AutoDiff max(AutoDiff a, const AutoDiff& b) {
  if (b.value() > a.value()) {
    a = b;
  }
  return a;
}

inline AutoDiff max(AutoDiff a, double b) {
  if (b > a.value()) {
    a = b;
  }
  return a;
}

inline AutoDiff max(double a, AutoDiff b) {
  if (a > b.value()) {
    b = a;
  }
  return b;
}

inline AutoDiff min(AutoDiff a, const AutoDiff& b) {
  if (b.value() < a.value()) {
    a = b;
  }
  return a;
}

inline AutoDiff min(AutoDiff a, double b) {
  if (b < a.value()) {
    a = b;
  }
  return a;
}

// NOLINTNEXTLINE(build/include_what_you_use) false positive.
inline AutoDiff min(double a, AutoDiff b) {
  if (a < b.value()) {
    b = a;
  }
  return b;
}

/** ADL overload to mimic std::pow from <cmath>. */
inline AutoDiff pow(AutoDiff a, const AutoDiff& b) {
  DRAKE_DEMAND(false);  // XXX implement me
  (void)(b);
  return a;
}

/** ADL overload to mimic std::pow from <cmath>. */
inline AutoDiff pow(AutoDiff a, double b) {
  a.partials().Mul(b * std::pow(a.value(), b - 1));
  a.value() = std::pow(a.value(), b);
  return a;
}

/** ADL overload to mimic std::pow from <cmath>. */
inline AutoDiff pow(double a, AutoDiff b) {
  DRAKE_DEMAND(false);  // XXX implement me
  (void)(a);
  return b;
}

/** ADL overload to mimic std::ceil from <cmath>. */
inline double ceil(const AutoDiff& x) {
  return std::ceil(x.value());
}

/** ADL overload to mimic std::floor from <cmath>. */
inline double floor(const AutoDiff& x) {
  return std::floor(x.value());
}

/** ADL overload to mimic std::nexttoward from <cmath>. */
inline double nexttoward(const AutoDiff& from, long double to) {
  return std::nexttoward(from.value(), to);
}

/** ADL overload to mimic std::round from <cmath>. */
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
  x.partials().Mul(2.0);
  x.value() *= x.value();
  return x;
}

// === Miscellaneous functions ===

/** Outputs the value() part of x to the stream. */
std::ostream& operator<<(std::ostream& s, const AutoDiff& x);

}  // namespace autodiff
}  // namespace drake
