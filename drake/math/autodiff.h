/// @file
/// Utilities for arithmetic on AutoDiffScalar.

#pragma once

#include <cmath>

#include <Eigen/Dense>
#include <unsupported/Eigen/AutoDiff>

#include "drake/math/gradient.h"

/// Overloads round to mimic std::round from <cmath>.
/// Must appear in global namespace so that ADL can select between this
/// implementation and the STL one.
template <typename DerType>
double round(const Eigen::AutoDiffScalar<DerType>& x) {
  return round(x.value());
}

/// Overloads floor to mimic std::floor from <cmath>.
/// Must appear in global namespace so that ADL can select between this
/// implementation and the STL one.
template <typename DerType>
double floor(const Eigen::AutoDiffScalar<DerType>& x) {
  return floor(x.value());
}

namespace drake {
namespace math {

template <typename Derived>
struct AutoDiffToValueMatrix {
  typedef typename Eigen::Matrix<typename Derived::Scalar::Scalar,
                                 Derived::RowsAtCompileTime,
                                 Derived::ColsAtCompileTime> type;
};

template <typename Derived>
typename AutoDiffToValueMatrix<Derived>::type autoDiffToValueMatrix(
    const Eigen::MatrixBase<Derived>& auto_diff_matrix) {
  typename AutoDiffToValueMatrix<Derived>::type ret(auto_diff_matrix.rows(),
                                                    auto_diff_matrix.cols());
  for (int i = 0; i < auto_diff_matrix.rows(); i++) {
    for (int j = 0; j < auto_diff_matrix.cols(); ++j) {
      ret(i, j) = auto_diff_matrix(i, j).value();
    }
  }
  return ret;
}

template <typename Derived>
struct AutoDiffToGradientMatrix {
  typedef typename Gradient<
      Eigen::Matrix<typename Derived::Scalar::Scalar,
                    Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>,
      Eigen::Dynamic>::type type;
};

template <typename Derived>
typename AutoDiffToGradientMatrix<Derived>::type autoDiffToGradientMatrix(
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

  typename AutoDiffToGradientMatrix<Derived>::type gradient(
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
