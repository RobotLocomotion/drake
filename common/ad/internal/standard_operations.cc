// Our header file doesn't stand on its own; it should only ever be included
// indirectly via auto_diff.h

/* clang-format off to disable clang-format-includes */
// NOLINTNEXTLINE(build/include)
#include "drake/common/ad/auto_diff.h"
/* clang-format on */

#include <ostream>

#include <fmt/format.h>

namespace drake {
namespace ad {

AutoDiff sin(AutoDiff x) {
  const double new_value = std::sin(x.value());
  // ∂/∂x sin(x) = cos(x)
  // The domain of sin & cos are identical; no need for special cases here.
  x.partials().Mul(std::cos(x.value()));
  x.value() = new_value;
  return x;
}

AutoDiff cos(AutoDiff x) {
  const double new_value = std::cos(x.value());
  // ∂/∂x cos(x) = -sin(x)
  // The domain of cos & sin are identical; no need for special cases here.
  x.partials().Mul(-std::sin(x.value()));
  x.value() = new_value;
  return x;
}

AutoDiff tan(AutoDiff x) {
  const double new_value = std::tan(x.value());
  // ∂/∂x tan(x) = sec²(x) = cos⁻²(x)
  // The mathematical tan() function has poles at (½ + n)π; however no common
  // floating-point representation is able to represent π/2 exactly, so there is
  // no argument value for which a pole error occurs, and so the domain of
  // std::tan & std::cos are identical (only non-finite values are disallowed).
  const double cos_x = std::cos(x.value());
  x.partials().Div(cos_x * cos_x);
  x.value() = new_value;
  return x;
}

AutoDiff asin(AutoDiff x) {
  const double new_value = std::asin(x.value());
  // ∂/∂x asin(x) = 1 / sqrt(1 - x²)
  // The domain of asin is [-1, 1], which is the same as sqrt(1-x²); no need
  // for special cases here.
  const double x2 = x.value() * x.value();
  x.partials().Div(std::sqrt(1 - x2));
  x.value() = new_value;
  return x;
}

AutoDiff acos(AutoDiff x) {
  const double new_value = std::acos(x.value());
  // ∂/∂x acos(x) = -1 / sqrt(1 - x²)
  // The domain of acos is [-1, 1], which is the same as sqrt(1-x²); no need
  // for special cases here.
  const double x2 = x.value() * x.value();
  x.partials().Div(-std::sqrt(1 - x2));
  x.value() = new_value;
  return x;
}

AutoDiff atan(AutoDiff x) {
  const double new_value = std::atan(x.value());
  // ∂/∂x atan(x) = 1 / (1 + x²)
  // The domain of atan includes everything except NaN, which will propagate
  // automatically via 1 + x²; no need for special cases here.
  const double x2 = x.value() * x.value();
  x.partials().Div(1 + x2);
  x.value() = new_value;
  return x;
}

AutoDiff atan2(AutoDiff a, const AutoDiff& b) {
  const double new_value = std::atan2(a.value(), b.value());
  // ∂/∂x atan2(a, b) = (ba' - ab')/(a² + b²)
  // The domain of atan2 includes everything except NaN, which will propagate
  // automatically via `norm`.
  // TODO(jwnimmer-tri) Handle the IEEE special cases for ±∞ and ±0 as input(s).
  // Figure out the proper gradients in that case.
  const double norm = a.value() * a.value() + b.value() * b.value();
  a.partials().Mul(b.value());
  a.partials().AddScaled(-a.value(), b.partials());
  a.partials().Div(norm);
  a.value() = new_value;
  return a;
}

AutoDiff atan2(AutoDiff a, double b) {
  const double new_value = std::atan2(a.value(), b);
  // ∂/∂x atan2(a, b) = (ba' - ab')/(a² + b²) = ba'/(a² + b²)
  // The domain of atan2 includes everything except NaN, which will propagate
  // automatically via `norm`.
  // TODO(jwnimmer-tri) Handle the IEEE special cases for ±∞ and ±0 as input(s).
  // Figure out the proper gradients in that case.
  const double norm = a.value() * a.value() + b * b;
  a.partials().Mul(b / norm);
  a.value() = new_value;
  return a;
}

AutoDiff atan2(double a, AutoDiff b) {
  const double new_value = std::atan2(a, b.value());
  // ∂/∂x atan2(a, b) = (ba' - ab')/(a² + b²) = -ab'/(a² + b²)
  // The domain of atan2 includes everything except NaN, which will propagate
  // automatically via `norm`.
  // TODO(jwnimmer-tri) Handle the IEEE special cases for ±∞ and ±0 as input(s).
  // Figure out the proper gradients in that case.
  const double norm = a * a + b.value() * b.value();
  b.partials().Mul(-a / norm);
  b.value() = new_value;
  return b;
}

AutoDiff sinh(AutoDiff x) {
  const double new_value = std::sinh(x.value());
  // ∂/∂x sinh(x) = cosh(x)
  // The domain of sinh & cosh are identical; no need for special cases here.
  x.partials().Mul(std::cosh(x.value()));
  x.value() = new_value;
  return x;
}

AutoDiff cosh(AutoDiff x) {
  const double new_value = std::cosh(x.value());
  // ∂/∂x cosh(x) = sinh(x)
  // The domain of cosh & sinh are identical; no need for special cases here.
  x.partials().Mul(std::sinh(x.value()));
  x.value() = new_value;
  return x;
}

AutoDiff tanh(AutoDiff x) {
  const double new_value = std::tanh(x.value());
  // ∂/∂x tanh(x) = 1 - tanh²(x)
  x.partials().Mul(1 - (new_value * new_value));
  x.value() = new_value;
  return x;
}

AutoDiff ceil(AutoDiff x) {
  x.value() = std::ceil(x.value());
  x.partials().SetZero();
  return x;
}

AutoDiff floor(AutoDiff x) {
  x.value() = std::floor(x.value());
  x.partials().SetZero();
  return x;
}

AutoDiff round(AutoDiff x) {
  x.value() = std::round(x.value());
  x.partials().SetZero();
  return x;
}

AutoDiff nexttoward(AutoDiff from, long double to) {
  from.value() = std::nexttoward(from.value(), to);
  from.partials().SetZero();
  return from;
}

std::ostream& operator<<(std::ostream& s, const AutoDiff& x) {
  return s << fmt::format("{}", x.value());
}

}  // namespace ad
}  // namespace drake
