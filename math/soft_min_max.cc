#include "drake/math/soft_min_max.h"

#include <algorithm>
#include <cmath>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace math {
namespace {
using std::exp;
using std::isfinite;
using std::log;

/* We use tare_x to exploit the shift invariance of log-sum-exp to avoid
overflow. It should be set to the extremum of x (i.e., max when alpha_prime > 0,
min when alpha_prime < 0). */
template <typename T>
T SoftOverMaxImpl(const std::vector<T>& x, double tare_x, double alpha_prime) {
  // Note that the function
  // f(x) = log(∑ᵢ exp(αxᵢ)) /α satisfies f(x) = f(x - m) + m for any arbitrary
  // m. Here we use m = tare_x.
  T sum_exp{0};
  for (const T& xi : x) {
    sum_exp += exp(alpha_prime * (xi - tare_x));
  }
  return log(sum_exp) / alpha_prime + tare_x;
}

/* We use tare_x to avoid overflow. It should be set to the extremum of x
(i.e., max when alpha_prime > 0, min when alpha_prime < 0). */
template <typename T>
T SoftUnderMaxImpl(const std::vector<T>& x, double tare_x, double alpha_prime) {
  // Note that the function
  // f(x) = ∑ᵢ exp(αxᵢ)*xᵢ / ∑ⱼ exp(αxⱼ) satisfies
  // f(x) = ∑ᵢ exp(α(xᵢ-m))*xᵢ / ∑ⱼ exp(α(xⱼ-m)) for any arbitrary m, by
  // multiplying exp(-αm) on both the numerator and denominator.
  T soft_max{0};
  T sum_exp{0};
  for (const T& xi : x) {
    const T exp_xi = exp(alpha_prime * (xi - tare_x));
    sum_exp += exp_xi;
    soft_max += exp_xi * xi;
  }
  return soft_max / sum_exp;
}
}  // namespace

template <typename T>
T SoftOverMax(const std::vector<T>& x, const double alpha) {
  DRAKE_THROW_UNLESS(x.size() > 0);
  DRAKE_THROW_UNLESS(alpha > 0);
  DRAKE_THROW_UNLESS(std::isfinite(alpha));
  const double max_x =
      ExtractDoubleOrThrow(*std::max_element(x.begin(), x.end()));
  return SoftOverMaxImpl(x, max_x, alpha);
}

template <typename T>
T SoftUnderMax(const std::vector<T>& x, const double alpha) {
  DRAKE_THROW_UNLESS(x.size() > 0);
  DRAKE_THROW_UNLESS(alpha > 0);
  DRAKE_THROW_UNLESS(std::isfinite(alpha));
  const double max_x =
      ExtractDoubleOrThrow(*std::max_element(x.begin(), x.end()));
  return SoftUnderMaxImpl(x, max_x, alpha);
}

template <typename T>
T SoftOverMin(const std::vector<T>& x, const double alpha) {
  DRAKE_THROW_UNLESS(x.size() > 0);
  DRAKE_THROW_UNLESS(alpha > 0);
  DRAKE_THROW_UNLESS(std::isfinite(alpha));
  // min(x) = -max(-x)
  const double min_x =
      ExtractDoubleOrThrow(*std::min_element(x.begin(), x.end()));
  return SoftUnderMaxImpl(x, min_x, -alpha);
}

template <typename T>
T SoftUnderMin(const std::vector<T>& x, const double alpha) {
  DRAKE_THROW_UNLESS(x.size() > 0);
  DRAKE_THROW_UNLESS(alpha > 0);
  DRAKE_THROW_UNLESS(std::isfinite(alpha));
  // min(x) = -max(-x)
  const double min_x =
      ExtractDoubleOrThrow(*std::min_element(x.begin(), x.end()));
  return SoftOverMaxImpl(x, min_x, -alpha);
}

// Explicit instantiation
DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&SoftOverMax<T>, &SoftUnderMax<T>, &SoftOverMin<T>, &SoftUnderMin<T>));
}  // namespace math
}  // namespace drake
