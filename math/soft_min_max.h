#pragma once

#include <cmath>
#include <algorithm>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_throw.h"

namespace drake {
namespace math {
namespace internal {
/** Computes log(∑exp(xᵢ)). Exploits the shift invariance of log-sum-exp to
avoid overflow. */
template <typename T>
T LogSumExp(const std::vector<T>& x) {
  DRAKE_ASSERT(x.size() > 0);
  using std::exp;
  using std::log;
  const T x_max = *std::max_element(x.begin(), x.end());
  T sum_exp{0.0};
  for (const T& xi : x) {
    sum_exp += exp(xi - x_max);
  }
  return x_max + log(sum_exp);
}
}  // namespace internal

/**
 Computes a smooth over approximation of max function, namely SoftOverMax(x) >=
 max(x).
 Mathematically we compute this as (log (∑ᵢ exp(αxᵢ))) / α.
 @param x The vector we want to compute its soft max.
 @param alpha α in the documentation above. Larger α makes the soft max more
 similar to max, with a sharper corner.
 @throws std::exception if α <= 0.
 */
template <typename T>
[[nodiscard]] T SoftOverMax(const std::vector<T>& x, double alpha = 1.0) {
  DRAKE_ASSERT(x.size() > 0);
  DRAKE_THROW_UNLESS(alpha > 0);
  if (alpha == 1.0) {
    return internal::LogSumExp(x);
  }
  std::vector<T> x_scaled{x};
  for (T& xi_scaled : x_scaled) {
    xi_scaled *= alpha;
  }
  return internal::LogSumExp(x_scaled) / alpha;
}

/**
 Computes a smooth under approximation of max function, namely SoftUnderMax(x)
 <= max(x). Mathematically we compute this as ∑ᵢ exp(αxᵢ)*xᵢ) / d, where d = ∑ⱼ
 exp(αxⱼ)
 @param x The vector we want to compute its soft max.
 @param alpha α in the documentation above. Larger α makes the soft max more
 similar to max, with a sharper corner.
 @throws std::exception if α <= 0.
 */
template <typename T>
[[nodiscard]] T SoftUnderMax(const std::vector<T>& x, double alpha = 1) {
  DRAKE_ASSERT(x.size() > 0);
  DRAKE_THROW_UNLESS(alpha > 0);
  std::vector<T> x_scaled{x};
  for (auto& xi_scaled : x_scaled) {
    xi_scaled *= alpha;
  }
  // To avoid overflow, we subtract x_scaled_max.
  const T x_scaled_max = *std::max_element(x_scaled.begin(), x_scaled.end());
  T soft_max{0};
  T d{0};
  for (int i = 0; i < static_cast<int>(x.size()); ++i) {
    using std::exp;
    const T exp_xi = exp(x_scaled[i] - x_scaled_max);
    soft_max += exp_xi * x[i];
    d += exp_xi;
  }
  return soft_max / d;
}

/**
 Computes a smooth over approximation of max function, namely SoftOverMin(x) >=
 min(x).
 Mathematically we compute this as ∑ᵢ exp(-αxᵢ)*xᵢ) / d, where d = ∑ⱼ
 exp(-αxⱼ)
 @param x The vector we want to compute its soft min.
 @param alpha α in the documentation above. Larger α makes the soft min more
 similar to min, with a sharper corner.
 @throws std::exception if α <= 0.
 */
template <typename T>
[[nodiscard]] T SoftOverMin(const std::vector<T>& x, double alpha = 1) {
  // min(x) = -max(-x)
  std::vector<T> x_negate{x};
  for (auto& xi : x_negate) {
    xi *= -1;
  }
  return -SoftUnderMax(x_negate, alpha);
}

template <typename T>
[[nodiscard]] T SoftUnderMin(const std::vector<T>& x, double alpha = 1) {
  // min(x) = -max(-x)
  std::vector<T> x_negate{x};
  for (auto& xi : x_negate) {
    xi *= -1;
  }
  return -SoftOverMax(x_negate, alpha);
}
}  // namespace math
}  // namespace drake
