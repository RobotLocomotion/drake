#pragma once

#include <limits>

#include <Eigen/Dense>

#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace math {
/** The 2-norm function |x| is not differentiable at x=0 (its gradient is
 x/|x|, which has a division-by-zero problem). On the other hand, x=0 happens
 very often. Hence we return a subgradient x/(|x| + ε) when x is
 almost 0, and returns the original gradient, x/|x|, otherwise. */
template <typename Derived>
typename Derived::Scalar DifferentiableNorm(
    const Eigen::MatrixBase<Derived>& x) {
  // We only support vectors for now.
  static_assert(Derived::ColsAtCompileTime == 1);

  const double kEps = std::numeric_limits<double>::epsilon();
  if constexpr (std::is_same_v<typename Derived::Scalar, AutoDiffXd>) {
    const Eigen::Matrix<double, Derived::RowsAtCompileTime,
                        Derived::ColsAtCompileTime>
        x_val = ExtractValue(x);
    const double norm_val = x_val.norm();
    if (norm_val > 100 * kEps) {
      return x.norm();
    } else {
      return AutoDiffXd(norm_val, ExtractGradient(x).transpose() * x_val /
                                      (norm_val + 10 * kEps));
    }
  } else {
    return x.norm();
  }
}
}  // namespace math
}  // namespace drake
