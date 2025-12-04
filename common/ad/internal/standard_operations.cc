// Our header file doesn't stand on its own; it should only ever be included
// indirectly via auto_diff.h

/* clang-format off to disable clang-format-includes */
// NOLINTNEXTLINE(build/include)
#include "drake/common/ad/auto_diff.h"
/* clang-format on */

#include <limits>
#include <ostream>
#include <utility>

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

AutoDiff exp(AutoDiff x) {
  // ∂/∂x eˣ = eˣ
  const double new_value = std::exp(x.value());
  x.partials().Mul(new_value);
  x.value() = new_value;
  return x;
}

AutoDiff log(AutoDiff x) {
  const double new_value = std::log(x.value());
  // ∂/∂x ln(x) = x⁻¹
  x.partials().Div(x.value());
  x.value() = new_value;
  return x;
}

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

AutoDiff pow(AutoDiff base, const AutoDiff& exp) {
  // Compute p = bˣ.
  const double b = base.value();
  const double x = exp.value();
  const double p = std::pow(b, x);

  // For the derivative, we have:
  //    ∂/∂v bˣ
  //  = ∂/∂v eˡⁿ⁽ᵇ⁾ˣ                      # Power via logarithms
  //  = eˡⁿ⁽ᵇ⁾ˣ ∂/∂v ln(b)x               # Derivative of exponentiation
  //  = bˣ ∂/∂v ln(b)x                    # Undo power via logarithms
  //  = bˣ (x ∂/∂v ln(b) + ln(b) ∂x/∂v)   # Multiplication chain rule
  //  = bˣ (xb⁻¹ ∂b/∂v + ln(b) ∂x/∂v)     # Derivative of logarithm
  //  = xbˣ⁻¹ ∂b/∂v + bˣln(b) ∂x/∂v       # Distribute
  //
  // We can write that as `db_scale * ∂b/∂v + dx_scale * ∂x/∂v`.
  double db_scale = x * std::pow(b, x - 1);
  double dx_scale = p * std::log(b);

  // However, there are several special cases that need customization:
  if (!std::isfinite(b) || !std::isfinite(x) || !std::isfinite(p)) {
    // It's impractical to choose well-defined derivatives of ±∞ and/or NaN.
    db_scale = kNaN;
    dx_scale = kNaN;
  } else if (b < 0) {
    // The result would be finite only if the exponent was an integer, which
    // means that any non-zero entries in ∂x/∂vᵢ make our resulting derivative
    // ill-defined (since an infinitesimal change would make change it from an
    // integer to a non-integer).
    dx_scale = kNaN;
  } else if (b == 0) {
    if (x == 0) {
      // The xbˣ⁻¹ would come out as a NaN (0 × ∞), but we can reason about the
      // limit for a better answer: the limit b→0 0/b is 0.
      db_scale = 0;
    } else if (x > 0) {
      // The bˣln(b) would come out as NaN (0 × -∞), but we can reason about the
      // limit for a better answer: for x > 0, the limit b→0⁺ bˣln(b) is 0.
      dx_scale = 0;
    }
  }

  // Assemble the result.
  AutoDiff result{p};
  result.partials() = std::move(base.partials());
  result.partials().Mul(db_scale);
  result.partials().AddScaled(dx_scale, exp.partials());
  return result;
}

AutoDiff pow(double base, const AutoDiff& exp) {
  return pow(AutoDiff{base}, exp);
}

AutoDiff pow(AutoDiff base, double exp) {
  return pow(std::move(base), AutoDiff{exp});
}

AutoDiff sqrt(AutoDiff x) {
  // ∂/∂x x¹ᐟ² = ½x⁻¹ᐟ² = 1/(2x¹ᐟ²)
  const double new_value = std::sqrt(x.value());
  x.partials().Div(2 * new_value);
  x.value() = new_value;
  return x;
}

AutoDiff copysign(const AutoDiff& mag, const AutoDiff& sgn) {
  return std::signbit(mag.value()) == std::signbit(sgn.value()) ? mag : -mag;
}

}  // namespace ad
}  // namespace drake
