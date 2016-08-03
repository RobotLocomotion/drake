/// @file
/// Utilities that relate simultaneously to both autodiff matrices and
/// gradient matrices.

#pragma once

#include <Eigen/Dense>

#include "drake/math/autodiff_gradient_internal.h"

namespace drake {
namespace math {

/// The return type is drake::math::Gradient, with the size and type
/// of Derived consistent with the parameter, Nq = Dynamic, and
/// DerivativeOrder = 1.
template <typename Derived>
auto autoDiffToGradientMatrix(
    const Eigen::MatrixBase<Derived>& auto_diff_matrix,
    int num_variables = Eigen::Dynamic) {
  int num_variables_from_matrix = 0;
  for (int i = 0; i < auto_diff_matrix.size(); ++i) {
    num_variables_from_matrix =
        std::max(num_variables_from_matrix,
                 static_cast<int>(auto_diff_matrix(i).derivatives().size()));
  }
  if (num_variables == Eigen::Dynamic) {
    num_variables = num_variables_from_matrix;
  } else if (num_variables_from_matrix != 0 &&
             num_variables_from_matrix != num_variables) {
    std::stringstream buf;
    buf << "Input matrix has derivatives w.r.t " << num_variables_from_matrix
        << " variables, whereas num_variables is " << num_variables << ".\n";
    buf << "Either num_variables_from_matrix should be zero, or it should "
           "match num_variables.";
    throw std::runtime_error(buf.str());
  }

  typename gradient_internal::AutoDiffToGradientMatrix<Derived>::type gradient(
      auto_diff_matrix.size(), num_variables);
  for (int row = 0; row < auto_diff_matrix.rows(); row++) {
    for (int col = 0; col < auto_diff_matrix.cols(); col++) {
      auto gradient_row =
          gradient.row(row + col * auto_diff_matrix.rows()).transpose();
      if (auto_diff_matrix(row, col).derivatives().size() == 0) {
        gradient_row.setZero();
      } else {
        gradient_row = auto_diff_matrix(row, col).derivatives();
      }
    }
  }
  return gradient;
}

}  // namespace math
}  // namespace drake
