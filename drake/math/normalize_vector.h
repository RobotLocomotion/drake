#pragma once

#include <Eigen/Dense>
#include "drake/math/gradient.h"
#include "drake/util/drakeGradientUtil.h"

namespace drake {
namespace math {
// NOTE: not reshaping second derivative to Matlab geval output format!
template <typename Derived>
void NormalizeVector(
    const Eigen::MatrixBase<Derived>& x, typename Derived::PlainObject& x_norm,
    typename drake::math::Gradient<Derived, Derived::RowsAtCompileTime,
                                   1>::type* dx_norm = nullptr,
    typename drake::math::Gradient<Derived, Derived::RowsAtCompileTime,
                                   2>::type* ddx_norm = nullptr) {
  typename Derived::Scalar xdotx = x.squaredNorm();
  typename Derived::Scalar norm_x = sqrt(xdotx);
  x_norm = x / norm_x;

  if (dx_norm) {
    dx_norm->setIdentity(x.rows(), x.rows());
    (*dx_norm) -= x * x.transpose() / xdotx;
    (*dx_norm) /= norm_x;

    if (ddx_norm) {
      auto dx_norm_transpose = transposeGrad(*dx_norm, x.rows());
      auto ddx_norm_times_norm = -matGradMultMat(x_norm, x_norm.transpose(),
                                                 (*dx_norm), dx_norm_transpose);
      auto dnorm_inv = -x.transpose() / (xdotx * norm_x);
      (*ddx_norm) = ddx_norm_times_norm / norm_x;
      auto temp = (*dx_norm) * norm_x;
      typename Derived::Index n = x.rows();
      for (int col = 0; col < n; col++) {
        auto column_as_matrix = (dnorm_inv(0, col) * temp);
        for (int row_block = 0; row_block < n; row_block++) {
          ddx_norm->block(row_block * n, col, n, 1) +=
              column_as_matrix.col(row_block);
        }
      }
    }
  }
}
} // namespace math
} // namespace drake