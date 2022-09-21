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

// Iterates over the partials of `input`. For any non-zero elements, sets the
// corresponding partial in `output` to NaN.
// @pre output->MatchSizeOf(input) has already been performed.
void UndefineOutputGradWhereInputGradNonzero(
    const AutoDiff& input, AutoDiff* output) {
  DRAKE_DEMAND(output != nullptr);

  // If either input or output are empty, there's nothing to do.
  if (output->partials().size() == 0) {
    // MatchSizeOf guarantees this.
    DRAKE_ASSERT(input.partials().size() == 0);
    return;
  }
  if (input.partials().size() == 0) {
    return;
  }

  // In case input and output are the same object, we need to call the
  // non-const member function first.
  auto& output_grad = output->partials().get_raw_storage_mutable();
  const auto& input_grad = input.partials().make_const_xpr();

  // Update the `output` per our API contract.
  const int size = output_grad.size();
  DRAKE_ASSERT(size == input_grad.size());
  for (int i = 0; i < size; ++i) {
    if (!(input_grad[i] == 0.0)) {
      output_grad[i] = kNaN;
    }
  }
}

}  // namespace

AutoDiff pow(AutoDiff base_ad, const AutoDiff& exp_ad) {
  // It's convenient to immediately set up the result, so we'll need to take a
  // picture of `base_ad.value()` first. Might as well do `exp` for convenience.
  const double base = base_ad.value();
  const double exp = exp_ad.value();

  // Result starts out holding the proper return value, but its partials are
  // just grad(base) to start.  We'll adjust them at the end; for now, just
  // set them to the correct size.
  AutoDiff result = std::move(base_ad);
  result.value() = std::pow(base, exp);
  result.partials().MatchSizeOf(exp_ad.partials());

  // If any of {base, exp, result} are NaN, then grad(result) is always NaN.
  if (std::isnan(result.value()) || std::isnan(base) || std::isnan(exp)) {
    result.partials().Mul(kNaN);
    return result;
  }

  // When dealing with infinities (or a zero base where a sign-change to the
  // exponent introduces infinities), trying to compute well-defined gradients
  // with appropriate symmetries is impractical. In that case, when grad(base)
  // and grad(exp) are both zero we'll leave grad(result) as zero, but otherwise
  // any non-zero input gradient becomes ill-defined in the result.
  if (base == 0 || !std::isfinite(base) || !std::isfinite(exp)) {
    UndefineOutputGradWhereInputGradNonzero(result, &result);
    UndefineOutputGradWhereInputGradNonzero(exp_ad, &result);
    return result;
  }

  // For the gradient, we have:
  //    ∂/∂v bˣ
  //  = ∂/∂v eˡⁿ⁽ᵇ⁾ˣ                      # Power via logarithms
  //  = eˡⁿ⁽ᵇ⁾ˣ ∂/∂v ln(b)x               # Derivative of exponentiation
  //  = bˣ ∂/∂v ln(b)x                    # Undo power via logarithms
  //  = bˣ (x ∂/∂v ln(b) + ln(b) ∂x/∂v)   # Multiplication chain rule
  //  = bˣ (xb⁻¹ ∂b/∂v + ln(b) ∂x/∂v)     # Derivative of logarithm
  //  = xbˣ⁻¹ ∂b/∂v + bˣln(b) ∂x/∂v       # Distribute

  // Account for the contribution of grad(base) on grad(result).
  // Don't try to compute (xbˣ⁻¹) with x == 0, in case it comes out as a NaN
  // (e.g., 0 * ∞). Instead, just assume that the x == 0 wins, such that the
  // grad(base) has no contribution to the result.
  const double base_grad_scale = (exp == 0) ? 0 : exp * std::pow(base, exp - 1);
  DRAKE_DEMAND(std::isfinite(base_grad_scale));
  result.partials().Mul(base_grad_scale);

  // Account for the contribution of grad(exp) on grad(result).
  if (base < 0) {
    UndefineOutputGradWhereInputGradNonzero(exp_ad, &result);
  } else {
    DRAKE_DEMAND(base > 0);
    const double exp_grad_scale = result.value() * std::log(base);
    DRAKE_DEMAND(std::isfinite(exp_grad_scale));
    result.partials().AddScaled(exp_grad_scale, exp_ad.partials());
  }

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
