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

AutoDiff exp(AutoDiff x) {
  // ∂/∂x eˣ = eˣ
  const double new_value = std::exp(x.value());
  x.partials().Mul(new_value);
  x.value() = new_value;
  return x;
}

AutoDiff log(AutoDiff x) {
  // ∂/∂x ln(x) = x⁻¹
  x.partials().Div(x.value());
  x.value() = std::log(x.value());
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
    DRAKE_DEMAND(input.partials().size() == 0);
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
  DRAKE_DEMAND(size == input_grad.size());
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
    return result;
  }

  // Rule #10: pow(base, ±0) returns 1 for any base, even when base is NaN.
  // We do not intercept positive finite `base` here, since those gradients
  // remain interesting.
  if (!(base > 0.0 && base < kInf) && exp == 0.0) {
    // The grad(result) is zero anywhere grad(exp) was zero, but otherwise is
    // ill-defined.
    UndefineOutputGradWhereInputGradNonzero(exp_ad, &result);
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
    // For #17, #19, #20, #21 the grad(result) is well-defined everywhere.
    // For #16 and #18, the grad(result) is ill-defined anywhere grad(exp) was
    // non-zero.
    result.partials().SetZero();
    if (std::signbit(result.value())) {
      UndefineOutputGradWhereInputGradNonzero(exp_ad, &result);
    }
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
    result.partials().SetZero();
    return result;
  }

  // Rule #1: pow(+0, exp), where exp is a negative odd integer, returns +∞.
  // Rule #2: pow(-0, exp), where exp is a negative odd integer, returns -∞.
  // Rule #3: pow(±0, exp), where exp is negative, finite, and is an even
  //   integer or a non-integer, returns +∞.
  // Rule #4: pow(±0, -∞) returns +∞.
  // Rule #5: pow(+0, exp), where exp is a positive odd integer, returns +0.
  // Rule #6: pow(-0, exp), where exp is a positive odd integer, returns -0.
  // Rule #7: pow(±0, exp), where exp is positive non-integer or a positive
  //   even integer, returns +0.
  if (base == 0) {
    // For #1, #3, #4, #5, #7 the grad(result) is well-defined everywhere.
    // For #2 and #6, the grad(result) is ill-defined anywhere grad(exp) was
    // non-zero.
    result.partials().SetZero();
    if (std::signbit(result.value())) {
      UndefineOutputGradWhereInputGradNonzero(exp_ad, &result);
    }
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
    UndefineOutputGradWhereInputGradNonzero(exp_ad, &result);
  } else {
    result.partials().AddScaled(exp_grad_scale, exp_ad.partials());
  }
  return result;
}

AutoDiff pow(double base, const AutoDiff& exp) {
  return pow(AutoDiff{base}, exp);
}

AutoDiff pow(AutoDiff base, double exp) {
  // Handle some fast-path cases.
  if (exp == -1) {
    return 1.0 / std::move(base);
  } else if (exp == 0) {
    base.value() = 1.0;
    base.partials().SetZero();
    return base;
  } else if (exp == 1) {
    return base;
  } else if (exp == 2) {
    base *= base;
    return base;
  }

  // Delegate to the full implementation.
  return pow(std::move(base), AutoDiff{exp});
}

AutoDiff sqrt(AutoDiff x) {
  // ∂/∂x x¹ᐟ² = ½x⁻¹ᐟ² = 1/(2x¹ᐟ²)
  const double new_value = std::sqrt(x.value());
  x.partials().Div(2 * new_value);
  x.value() = new_value;
  return x;
}

std::ostream& operator<<(std::ostream& s, const AutoDiff& x) {
  return s << fmt::format("{}", x.value());
}

}  // namespace ad
}  // namespace drake
