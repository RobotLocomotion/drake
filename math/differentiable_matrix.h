/** @file
A matrix and its derivatives. */

#pragma once

#include <cmath>
#include <type_traits>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace math {

template <typename MatrixType, bool with_derivatives>
class DifferentiableMatrix { };

template <typename MatrixType>
class DifferentiableMatrix<MatrixType, false> : public MatrixType {
};

/** MatrixType must have double scalars. */
template <typename MatrixType>
class DifferentiableMatrix<MatrixType, true> {
  static_assert(std::is_same_v<typename MatrixType::Scalar, double>);

 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DifferentiableMatrix);

  explicit DifferentiableMatrix(const MatrixType& value) : value_(value) {}

  const MatrixType& value() const { return value_; }

  using MatrixTransposeType = Eigen::Matrix<
      double, MatrixType::ColsAtCompileTime, MatrixType::RowsAtCompileTime,
      0, MatrixType::MaxColsAtCompileTime, MatrixType::MaxRowsAtCompileTime>;

  using TransposeType = DifferentiableMatrix<MatrixTransposeType, true>;

  using AutoDiffMatrixType = Eigen::Matrix<
      AutoDiffXd, MatrixType::RowsAtCompileTime, MatrixType::ColsAtCompileTime,
      0, MatrixType::MaxRowsAtCompileTime, MatrixType::MaxColsAtCompileTime>;

  using GradientMatrixType = Eigen::Matrix<
      double, MatrixType::SizeAtCompileTime, Eigen::Dynamic>;

  GradientMatrixType ExtractGradient() const {
    const int sz = value_.size();
    GradientMatrixType gradient =
        GradientMatrixType::Zero(sz, max_num_derivs_);

    for (int n=0; n < static_cast<int>(non_zeros_.size()); ++n) {
      const int d = non_zeros_[n];
      const MatrixType& deriv = derivatives_[n];
      for (int i=0; i < sz; ++i)
        gradient(i, d) = deriv(i);
    }

    return gradient;
  }

  explicit DifferentiableMatrix(const AutoDiffMatrixType& ad) {
    const int nr = ad.rows();
    const int nc = ad.cols();
    const int sz = nr * nc;

    // Derivative vectors in ad are either zero length or all the same
    // maximum length. So we only need to find the first non-zero length one.
    // Zero-length derivatives are rare, so we have a good shot at just looking
    // at one entry here.
    for (int i=0; i < sz; ++i) {
      const AutoDiffXd& entry = ad(i);
      if ((max_num_derivs_ = entry.derivatives().size()) > 0)
        break;
    }

    // Next let's find the non-zeros. First we'll just put a "0" in any
    // spot corresponding to a non-zero derivative, then we'll replace
    // that with the allocated slot in the derivatives vector.
    //
    // We can stop early if the number of non-zeros reaches the maximum.
    std::vector<int> deriv_index(max_num_derivs_, -1);  // heap allocation
    int num_non_zeros = 0;
    for (int i=0; i < sz && num_non_zeros < max_num_derivs_; ++i) {
      const AutoDiffXd& entry = ad(i);
      const Eigen::VectorXd& derivs = entry.derivatives();
      if (derivs.size() == 0)
        continue;  // That can't contribute any non-zeros.
      DRAKE_ASSERT(static_cast<int>(derivs.size()) == max_num_derivs_);
      for (int d=0; d < max_num_derivs_; ++d) {
        if (deriv_index[d] == 0 || derivs[d] == 0.0) continue;
        deriv_index[d] = 0;
        ++num_non_zeros;
      }
    }

    // We know which of the derivatives are non-zero. Next, assign memory
    // to each of those and build the non-zero table and replace each of
    // the non-zero bits with the actual assigned memory index.
    non_zeros_.reserve(num_non_zeros);  // heap allocation
    for (int d=0; d < max_num_derivs_; ++d) {
      if (deriv_index[d] == 0) {
        deriv_index[d] = non_zeros_.size();  // Next available spot.
        non_zeros_.push_back(d);
      }
    }
    DRAKE_DEMAND(static_cast<int>(non_zeros_.size()) == num_non_zeros);

    // Finally, fill in the data for the value and derivatives.
    value_.resize(nr, nc);  // Will do nothing for fixed-size matrix.
    derivatives_.resize(num_non_zeros);
    for (int n=0; n < num_non_zeros; ++n)
      derivatives_[n].resize(nr, nc);
    for (int i=0; i < sz; ++i) {
      const AutoDiffXd& entry = ad(i);
      const Eigen::VectorXd& derivs = entry.derivatives();
      value_(i) = entry.value();
      if (derivs.size() == 0) {
        for (int n=0; n < num_non_zeros; ++n)
          derivatives_[n](i) = 0.0;
        continue;
      }
      DRAKE_ASSERT(static_cast<int>(derivs.size()) == max_num_derivs_);
      for (int n=0; n < num_non_zeros; ++n)
        derivatives_[n](i) = derivs(non_zeros_[n]);
    }
  }

  AutoDiffMatrixType ToAutoDiffXd() const {
    const int nr = value_.rows();
    const int nc = value_.cols();
    const int sz = nr * nc;
    AutoDiffMatrixType ad(nr, nc);
    for (int i=0; i < sz; ++i) {
      ad(i).value() = value_(i);
      ad(i).derivatives() = Eigen::VectorXd::Zero(max_num_derivs_);
    }

    for (int n=0; n < static_cast<int>(non_zeros_.size()); ++n) {
      const MatrixType& deriv = derivatives_[n];
      for (int i=0; i < sz; ++i)
        ad(i).derivatives()(non_zeros_[n]) = deriv(i);
    }

    return ad;
  }

  TransposeType transpose() const {
    TransposeType matrix_transpose(
        max_num_derivs_, non_zeros_);
    matrix_transpose.SetValue(value_.transpose());
    for (int n = 0; n < static_cast<int>(non_zeros_.size()); ++n)
      matrix_transpose.SetNonzeroDerivative(n, derivatives_[n].transpose());
    return matrix_transpose;
  }

 private:
  friend TransposeType;
  explicit DifferentiableMatrix(int max_num_derivatives,
                                const std::vector<int>& non_zeros)
      : max_num_derivs_(max_num_derivatives),
      non_zeros_(non_zeros){
    DRAKE_DEMAND(max_num_derivatives >= 0);
    DRAKE_DEMAND(static_cast<int>(non_zeros.size()) <= max_num_derivatives);
    derivatives_.resize(non_zeros.size());
  }

  void SetValue(MatrixType&& source) {
    value_ = std::move(source);
  }

  void SetNonzeroDerivative(int n, MatrixType&& source) {
    DRAKE_DEMAND(0 <= n && n < static_cast<int>(non_zeros_.size()));
    derivatives_[n] = std::move(source);
  }

  MatrixType value_;
  int max_num_derivs_{0};
  std::vector<int> non_zeros_;
  std::vector<MatrixType> derivatives_;
};

template <typename MatrixType>
const MatrixType& ExtractValue(
    const DifferentiableMatrix<MatrixType, true>& matrix) {
  return matrix.value();
}

template <typename MatrixType>
const typename DifferentiableMatrix<MatrixType, true>::GradientMatrixType
ExtractGradient(const DifferentiableMatrix<MatrixType, true>& matrix) {
  return matrix.ExtractGradient();
}

}  // namespace math
}  // namespace drake
