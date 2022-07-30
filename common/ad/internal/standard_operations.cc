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
  // ∂/∂x sin(x) = cos(x)
  x.partials().Mul(std::cos(x.value()));
  x.value() = std::sin(x.value());
  return x;
}

AutoDiff cos(AutoDiff x) {
  // ∂/∂x cos(x) = -sin(x)
  x.partials().Mul(-std::sin(x.value()));
  x.value() = std::cos(x.value());
  return x;
}

AutoDiff tan(AutoDiff x) {
  // ∂/∂x tan(x) = sec²(x) = cos⁻²(x)
  const double cos_x = std::cos(x.value());
  x.partials().Div(cos_x * cos_x);
  x.value() = std::tan(x.value());
  return x;
}

AutoDiff asin(AutoDiff x) {
  // ∂/∂x acos(x) = 1 / sqrt(1 - x²)
  const double x2 = x.value() * x.value();
  x.partials().Div(std::sqrt(1 - x2));
  x.value() = std::asin(x.value());
  return x;
}

AutoDiff acos(AutoDiff x) {
  // ∂/∂x acos(x) = -1 / sqrt(1 - x²)
  const double x2 = x.value() * x.value();
  x.partials().Div(-std::sqrt(1 - x2));
  x.value() = std::acos(x.value());
  return x;
}

AutoDiff atan(AutoDiff x) {
  // ∂/∂x atan(x) = 1 / (1 + x²)
  const double x2 = x.value() * x.value();
  x.partials().Div(1 + x2);
  x.value() = std::atan(x.value());
  return x;
}

AutoDiff atan2(AutoDiff a, const AutoDiff& b) {
  // ∂/∂x atan2(a, b) = (ba' - ab')/(a² + b²)
  const double norm = a.value() * a.value() + b.value() * b.value();
  a.partials().Mul(b.value());
  a.partials().AddScaled(-a.value(), b.partials());
  a.partials().Div(norm);
  a.value() = std::atan2(a.value(), b.value());
  return a;
}

AutoDiff atan2(AutoDiff a, double b) {
  // ∂/∂x atan2(a, b) = (ba' - ab')/(a² + b²) = ba'/(a² + b²)
  const double norm = a.value() * a.value() + b * b;
  a.partials().Mul(b / norm);
  a.value() = std::atan2(a.value(), b);
  return a;
}

AutoDiff atan2(double a, AutoDiff b) {
  // ∂/∂x atan2(a, b) = (ba' - ab')/(a² + b²) = -ab'/(a² + b²)
  const double norm = a * a + b.value() * b.value();
  b.partials().Mul(-a / norm);
  b.value() = std::atan2(a, b.value());
  return b;
}

AutoDiff sinh(AutoDiff x) {
  // ∂/∂x sinh(x) = cosh(x)
  x.partials().Mul(std::cosh(x.value()));
  x.value() = std::sinh(x.value());
  return x;
}

AutoDiff cosh(AutoDiff x) {
  // ∂/∂x cosh(x) = sinh(x)
  x.partials().Mul(std::sinh(x.value()));
  x.value() = std::cosh(x.value());
  return x;
}

AutoDiff tanh(AutoDiff x) {
  // ∂/∂x tanh(x) = 1 - tanh²(x)
  const double new_value = std::tanh(x.value());
  x.partials().Mul(1 - (new_value * new_value));
  x.value() = new_value;
  return x;
}

std::ostream& operator<<(std::ostream& s, const AutoDiff& x) {
  return s << fmt::format("{}", x.value());
}

}  // namespace ad
}  // namespace drake
