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
constexpr double kInf = std::numeric_limits<double>::infinity();

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

  // Refer to https://en.cppreference.com/w/cpp/numeric/math/pow for all of the
  // special cases we need to handle here before we move on to real-number math.

  // We'll start by dealing with NaNs.

  // For starters, if we see a NaN result, then we'll NaN its gradients as well.
  // - Rule #11: pow(base, exp) returns NaN if base is finite and negative and
  //     exp is finite and non-integer.
  // - Rule #22: [other than #9 and #10], if any argument is NaN, NaN is
  //     returned.
  if (std::isnan(result.value())) {
    result.partials().Mul(kNaN);
    return result;
  }

  // Rule #8: pow(-1, ±∞) returns 1.
  // Rule #9: pow(+1, exp) returns 1 for any exp, even when exp is NaN. We do
  // not intercept finite `exp` here, since those gradients remain interesting.
  if (std::abs(base) == 1.0 && !std::isfinite(exp)) {
    // The grad(result) remains zero anywhere grad(base) was zero, but
    // otherwise is ill-defined.
    UndefineOutputGradWhereInputGradNonzero(result, &result);
    result.partials().Mul(0.0);
    return result;
  }

  // Rule #10: pow(base, ±0) returns 1 for any base, even when base is NaN.
  // We do not intercept positive finite `base` here, since those gradients
  // remain interesting.
  if (!(base > 0.0 && base < kInf) && exp == 0.0) {
    // The grad(result) is zero anywhere grad(exp) was zero, but otherwise is
    // ill-defined.
    UndefineOutputGradWhereInputGradNonzero(exp_ad, &result);
    result.partials().Mul(0.0);
    return result;
  }

  // As of this point, all NaNs have been handled.
  DRAKE_DEMAND(!std::isnan(base));
  DRAKE_DEMAND(!std::isnan(exp));
  DRAKE_DEMAND(!std::isnan(result.value()));

  // Now we'll deal with infinities.

  // Rule #16: pow(-∞, exp) returns -0 if exp is a negative odd integer.
  // Rule #17: pow(-∞, exp) returns +0 if exp is a negative non-integer or
  //   negative even integer.
  // Rule #18: pow(-∞, exp) returns -∞ if exp is a positive odd integer.
  // Rule #19: pow(-∞, exp) returns +∞ if exp is a positive non-integer or
  //   positive even integer.
  // Rule #20: pow(+∞, exp) returns +0 for any negative exp.
  // Rule #21: pow(+∞, exp) returns +∞ for any positive exp.
  if (std::isinf(base)) {
    DRAKE_ASSERT(!std::isnan(exp));  // Rule #22 already matched, above.
    DRAKE_ASSERT(exp != 0);          // Rule #10 already matched, above.
    // For #16, #17, #19, #20, #21 the grad(result) is well-defined everywhere.
    // For #18, the grad(result) is ill-defined anywhere grad(exp) was non-zero.
    if (result.value() == -kInf) {
      UndefineOutputGradWhereInputGradNonzero(exp_ad, &result);
    }
    result.partials().Mul(0.0);
    return result;
  }

  // Rule #4: pow(±0, -∞) returns +∞.
  // Rule #12: pow(base, -∞) returns +∞ for any |base|<1.
  // Rule #14: pow(base, +∞) returns +0 for any |base|<1.
  // Rule #13: pow(base, -∞) returns +0 for any |base|>1.
  // Rule #15: pow(base, +∞) returns +∞ for any |base|>1.
  if (std::isinf(exp)) {
    DRAKE_ASSERT(base != 1.0);  // Rule #8 or #9 already matched, above.
    // The resulting gradient is well-defined everywhere.
    result.partials().Mul(0.0);
    return result;
  }

  // Rule #1: pow(+0, exp), where exp is a negative odd integer, returns +∞.
  // Rule #2: pow(-0, exp), where exp is a negative odd integer, returns -∞.
  // Rule #3: pow(±0, exp), where exp is negative, finite, and is an even
  //   integer or a non-integer, returns +∞.
  // Rule #5: pow(+0, exp), where exp is a positive odd integer, returns +0.
  // Rule #6: pow(-0, exp), where exp is a positive odd integer, returns -0.
  // Rule #7: pow(±0, exp), where exp is positive non-integer or a positive
  //   even integer, returns +0.
  if (base == 0) {
    // For #5, #6, #7 the grad(result) is well-defined everywhere.
    // For #1, #2, #3, the grad(result) is ill-defined anywhere grad(base) was
    // non-zero.
    if (std::isinf(result.value())) {
      UndefineOutputGradWhereInputGradNonzero(result, &result);
      // For #2 (only), the grad(result) is ill-defined anywhere grad(exp) was
      // non-zero.
      if (result.value() == -kInf) {
        UndefineOutputGradWhereInputGradNonzero(exp_ad, &result);
      }
    }
    result.partials().Mul(0.0);
    return result;
  }

  // As of this point, all infinities and NaNs have been handled.
  DRAKE_DEMAND(std::isfinite(base));
  DRAKE_DEMAND(std::isfinite(exp));
  DRAKE_DEMAND(std::isfinite(result.value()));

  // For the gradient, we have:
  //  ∂/∂x aᵇ = aᵇ⁻¹ (b a' + a ln(a) b')
  const double base_grad_scale = std::pow(base, exp - 1) * exp;
  const double exp_grad_scale = result.value() * std::log(base);
  result.partials().Mul(base_grad_scale);
  if (std::isnan(exp_grad_scale)) {
    DRAKE_ASSERT(base < 0.0);
    // When base is negative, only integer exponents are sensible, so the
    // grad(result) is ill-defined anywhere grad(exp) was non-zero.
    UndefineOutputGradWhereInputGradNonzero(exp_ad, &result);
  } else {
    // Note that if base == 1.0, the exp_grad_scale here will be 0.0.
    result.partials().AddScaled(exp_grad_scale, exp_ad.partials());
  }
  return result;
}

AutoDiff pow(double base, const AutoDiff& exp) {
  return pow(AutoDiff{base}, exp);
}

AutoDiff pow(AutoDiff base, double exp) {
  // Handle some fast-path cases.
  if (base.value() != 0.0 && std::isfinite(base.value())) {
    if (exp == -1) {
      return 1.0 / std::move(base);
    } else if (exp == 0) {
      base.value() = 1.0;
      base.partials().Mul(0.0);
      return base;
    } else if (exp == 1) {
      return base;
    } else if (exp == 2) {
      base *= base;
      return base;
    }
  }

  // Otherwise, delegate to the full implementation.
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
