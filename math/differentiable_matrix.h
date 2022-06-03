/** @file
A matrix and its derivatives. */

#pragma once

#include <algorithm>
#include <cmath>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace math {

// We're going to have to check that LHS ncol = RHS nrow (at least at runtime)
// and produce a result that is LHS nrow X RHS ncol, with constant dimensions
// if possible.
template <typename MatrixLhs, typename MatrixRhs>
struct DifferentiableMatrixMultiply {
  static constexpr int Lrows = MatrixLhs::RowsAtCompileTime;
  static constexpr int Lcols = MatrixLhs::ColsAtCompileTime;
  static constexpr int Rrows = MatrixRhs::RowsAtCompileTime;
  static constexpr int Rcols = MatrixRhs::ColsAtCompileTime;

  static constexpr bool need_runtime_shape_test =
    Lcols == Eigen::Dynamic || Rrows == Eigen::Dynamic;

  // Shapes are compatible if one or both dimensions are Dynamic or if they
  // are the same size at compile time.
  static constexpr bool shapes_are_compatible =
      Lcols == Rrows || need_runtime_shape_test;

  static bool ShapesAreConforming(const MatrixLhs& lhs,
                                  const MatrixRhs& rhs) {
    if constexpr (!shapes_are_compatible) {
      return false;
    } else {
      return lhs.cols() == rhs.rows();
    }
  }

  using MatrixResultType = Eigen::Matrix<double,
    Lrows, Rcols, 0, MatrixLhs::MaxRowsAtCompileTime,
    MatrixRhs::MaxColsAtCompileTime>;

  using AutoDiffMatrixResultType = Eigen::Matrix<AutoDiffXd,
    Lrows, Rcols, 0, MatrixLhs::MaxRowsAtCompileTime,
    MatrixRhs::MaxColsAtCompileTime>;
};

/** An Eigen matrix and its derivatives, specialized for `Matrix<AutoDiffXd>`.
For other scalar types, behaves like the Eigen matrix. */
template <typename MatrixT, typename T = typename MatrixT::Scalar>
class DifferentiableMatrix : public MatrixT {
  static_assert(std::is_same_v<typename MatrixT::Scalar, T>);
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DifferentiableMatrix);
  using MatrixT::MatrixT;

  using MatrixType = MatrixT;

  using MatrixTransposeType = Eigen::Matrix<
    T, MatrixT::ColsAtCompileTime, MatrixT::RowsAtCompileTime,
    0, MatrixT::MaxColsAtCompileTime, MatrixT::MaxRowsAtCompileTime>;

  /** This the type produced by the transpose() method. */
  using TransposeType = DifferentiableMatrix<MatrixTransposeType>;

  static constexpr bool is_eigen_matrix() { return true; }

  /** Returns the matrix value, excluding derivatives if any. */
  const MatrixType& value() const { return *this; }

  TransposeType transpose() const {
    return TransposeType(value().transpose());
  }

  /** Returns a mutable reference to the Eigen matrix. Note that this method is
  not available for the AutoDiffXd specialization. */
  MatrixType& mutable_value() { return *this; }

  /** This is the return type for a method that returns the Eigen matrix
  equivalent of this DifferentiableMatrix. For this T≠AutoDiffXd case, we can
  just return a reference. */
  using EigenEquivalentType = const MatrixT&;
  using EigenRowType = decltype(static_cast<const MatrixT*>(nullptr)->row(0));
  using EigenColType = decltype(static_cast<const MatrixT*>(nullptr)->col(0));

  EigenEquivalentType GetEigenEquivalent() const { return value(); }

  EigenRowType GetEigenRow(int i) const { return value().row(i); }
  EigenColType GetEigenCol(int j) const { return value().col(j); }
};

/** MatrixType must have AutoDiffXd scalars. */
template <typename MatrixT>
class DifferentiableMatrix<MatrixT, AutoDiffXd> {
  static_assert(std::is_same_v<typename MatrixT::Scalar, AutoDiffXd>);

 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DifferentiableMatrix);

  static constexpr bool is_eigen_matrix() { return false; }

  /** A matrix exactly like MatrixT but with the AutoDiffXd scalars replaced
  by doubles. */
  using MatrixType = Eigen::Matrix<
      double, MatrixT::RowsAtCompileTime, MatrixT::ColsAtCompileTime,
      0, MatrixT::MaxRowsAtCompileTime, MatrixT::MaxColsAtCompileTime>;

  using MatrixTransposeType = Eigen::Matrix<
      double, MatrixT::ColsAtCompileTime, MatrixT::RowsAtCompileTime,
      0, MatrixT::MaxColsAtCompileTime, MatrixT::MaxRowsAtCompileTime>;

  using AutoDiffMatrixType = MatrixT;

  using AutoDiffRowType = Eigen::Matrix<
      AutoDiffXd, 1, MatrixT::ColsAtCompileTime,
      Eigen::RowMajor, 1, MatrixT::MaxColsAtCompileTime>;

  using AutoDiffColType = Eigen::Matrix<
      AutoDiffXd, MatrixT::RowsAtCompileTime, 1,
      0, MatrixT::MaxRowsAtCompileTime, 1>;

  using AutoDiffMatrixTransposeType = Eigen::Matrix<
      AutoDiffXd, MatrixT::ColsAtCompileTime, MatrixT::RowsAtCompileTime,
      0, MatrixT::MaxColsAtCompileTime, MatrixT::MaxRowsAtCompileTime>;

  /** Constructs with an uninitialized matrix and no derivatives. */
  DifferentiableMatrix() {}

  /** Constructs a DifferentiableMatrix from a given `Matrix<AutoDiffXd>`.
  Note that this requires an exact type match to the template parameter
  MatrixT. */
  explicit DifferentiableMatrix(const AutoDiffMatrixType& ad);

  /** Sets this %DifferentiableMatrix from a given `Matrix<AutoDiffXd>`.
  Note that this requires an exact type match to the template parameter
  MatrixT. */
  DifferentiableMatrix& operator=(const AutoDiffMatrixType& ad);

  /** Returns the stored value with scalars of type double. */
  const MatrixType& value() const { return value_; }

  /** This is the return type for a method that returns the Eigen matrix
  equivalent of this DifferentiableMatrix. For this T=AutoDiffXd case, we must
  perform an expensive operation. */
  using EigenEquivalentType = AutoDiffMatrixType;
  using EigenRowType = AutoDiffRowType;
  using EigenColType = AutoDiffColType;

  /** Generate a Matrix<AutoDiffXd> equivalent to this DifferentialMatrix.
  This is expensive for this specialization and should be avoided. */
  EigenEquivalentType GetEigenEquivalent() const {
    return ToAutoDiffXd();
  }

  EigenRowType GetEigenRow(int i) const {
    // TODO(sherm1) Create just the row.
    EigenEquivalentType whole_matrix = GetEigenEquivalent();
    return whole_matrix.row(i);
  }

  EigenColType GetEigenCol(int j) const {
    // TODO(sherm1) Create just the column.
    EigenEquivalentType whole_matrix = GetEigenEquivalent();
    return whole_matrix.col(j);
  }

  /** Returns the nᵗʰ non-zero derivative. The corresponding variable is
  xᵥ, where v = non_zero(n). That is, we're returning dₙ = ∂M/∂xᵥ, where M
  is the matrix result returned by value(). */
  const MatrixType& derivative(int n) const {
    DRAKE_ASSERT(0 <= n && n < static_cast<int>(derivatives_.size()));
    return derivatives_[n];
  }

  /** Returns the variable number v for the nᵗʰ non-zero derivative. That is,
  the nᵗʰ non-zero derivative dₙ of this matrix M is dₙ = ∂M/∂xᵥ. */
  int non_zero(int n) const {
    DRAKE_ASSERT(0 <= n && n < num_non_zeros());
    return non_zeros_[n];
  }

  int num_non_zeros() const { return static_cast<int>(non_zeros_.size()); }
  int num_variables() const { return num_variables_; }


  /** This the type produced by the transpose() method. */
  using TransposeType = DifferentiableMatrix<AutoDiffMatrixTransposeType>;

  template <typename RhsMatrixType>
  using MultiplyResultType = DifferentiableMatrix<
      typename DifferentiableMatrixMultiply<
      MatrixType, RhsMatrixType>::AutoDiffMatrixResultType>;


  using GradientMatrixType = Eigen::Matrix<
      double, MatrixType::SizeAtCompileTime, Eigen::Dynamic>;

  /** Returns a gradient matrix in AutoDiff style -- one row per matrix
  element e, with column(v)=∂e/∂xᵥ. */
  GradientMatrixType ExtractGradient() const;


  /** Returns a `Matrix<AutoDiffXd>` object numerically equivalent to this
  DifferentiableMatrix object. */
  AutoDiffMatrixType ToAutoDiffXd() const;

  /** Returns a DifferentiableMatrix whose value and derivatives are the
  transpose of this object. */
  TransposeType transpose() const;

  /** Multiplies two DifferentiableMatrix objects and returns a new one that
  contains the product and its derivatives. The inputs must be conformable. */
  template <typename RhsMatrixType>
  MultiplyResultType<RhsMatrixType> operator*(
      const DifferentiableMatrix<RhsMatrixType, AutoDiffXd>& rhs) const;

 private:
  friend TransposeType;
  template <typename OtherMatrixType, typename>
  friend class DifferentiableMatrix;

  DifferentiableMatrix(int num_variables, const std::vector<int>& non_zeros)
      : DifferentiableMatrix(num_variables, std::vector<int>(non_zeros)) {}

  DifferentiableMatrix(int num_variables, std::vector<int>&& non_zeros)
      : num_variables_(num_variables), non_zeros_(std::move(non_zeros)) {
    DRAKE_DEMAND(num_variables >= 0);
    DRAKE_DEMAND(num_non_zeros() <= num_variables);
    derivatives_.resize(non_zeros_.size());
  }

  void SetValue(MatrixType&& source) {
    value_ = std::move(source);
  }

  void SetNonzeroDerivative(int n, MatrixType&& source) {
    DRAKE_DEMAND(0 <= n && n < static_cast<int>(non_zeros_.size()));
    derivatives_[n] = std::move(source);
  }

  MatrixType value_;
  int num_variables_{0};
  // These two vectors are the same size.
  std::vector<int> non_zeros_;  // Strictly increasing order.
  std::vector<MatrixType> derivatives_;
};

/** Returns a reference to the value contained in the given
DifferentiableMatrix. This works for any scalar type. */
template <typename MatrixT>
const typename DifferentiableMatrix<MatrixT>::MatrixType&
ExtractValue(const DifferentiableMatrix<MatrixT>& matrix) {
  return matrix.value();
}

/** Returns a gradient matrix in AutoDiff-compatible format that is numerically
equivalent to the derivatives stored in this DifferentiableMatrix. This is an
expensive conversion operation; you can access the derivatives directly via
the `derivative()` method of DifferentiableMatrix.
@see DifferentiableMatrix::derivative() */
template <typename MatrixT>
typename DifferentiableMatrix<MatrixT, AutoDiffXd>::GradientMatrixType
ExtractGradient(const DifferentiableMatrix<MatrixT, AutoDiffXd>& matrix) {
  return matrix.ExtractGradient();
}

/** Outputs a representation of a DifferentiableMatrix in human-readable form.
*/
template <typename MatrixT>
std::ostream& operator<<(std::ostream& o,
    const DifferentiableMatrix<MatrixT, AutoDiffXd>& matrix) {
  o << "x=\n" << matrix.value() << "\n";
  int nz_nxt = 0;
  for (int i=0; i < matrix.num_variables(); ++i) {
    o << "∂x/∂v_" << i << " =";
    if (i != matrix.non_zero(nz_nxt))
      o << " ZERO\n";
    else
      o << "\n" << matrix.derivative(nz_nxt++) << "\n";
  }
  return o;
}

// Definitions of DifferentiableMatrix class methods must be in the header
// since the class is arbitrarily templatized.

template <typename MatrixT>
DifferentiableMatrix<MatrixT, AutoDiffXd>::DifferentiableMatrix(
    const AutoDiffMatrixType& ad) {
  const int nr = ad.rows();
  const int nc = ad.cols();
  const int sz = nr * nc;

  // Derivative vectors in ad are either zero length or all the same
  // length nv=number of variables. So we only need to find the first
  // derivative vector that has a non-zero length. Zero-length derivatives
  // are rare, so we have a good shot at just looking at one entry here.
  for (int i = 0; i < sz; ++i) {
    const AutoDiffXd& entry = ad(i);
    // This is an assignment, not an equality test!
    if ((num_variables_ = entry.derivatives().size()) > 0) break;
  }

  // Next let's find the non-zeros. We'll put a "1" in any spot corresponding
  // to a non-zero derivative. We can stop early if the number of non-zeros
  // reaches the number of variables.

  // Logically this is a vector<bool> (but that's a weird specialization).
  std::vector<int> deriv_index(num_variables_, false);  // heap allocation
  int num_non_zeros = 0;
  for (int i = 0; i < sz && num_non_zeros < num_variables_; ++i) {
    const AutoDiffXd& entry = ad(i);
    const Eigen::VectorXd& derivs = entry.derivatives();
    if (derivs.size() == 0) continue;  // That can't contribute any non-zeros.
    DRAKE_ASSERT(static_cast<int>(derivs.size()) == num_variables_);
    for (int v = 0; v < num_variables_; ++v) {
      if (deriv_index[v] || derivs[v] == 0.0) continue;
      deriv_index[v] = true;  // This is a flag meaning "we saw a non-zero".
      ++num_non_zeros;
    }
  }

  // We know which of the derivatives are non-zero, so we can build the
  // non-zero table (whose entries must be sorted in increasing order).
  non_zeros_.reserve(num_non_zeros);  // heap allocation
  for (int v = 0; v < num_variables_; ++v)
    if (deriv_index[v]) non_zeros_.push_back(v);
  DRAKE_DEMAND(static_cast<int>(non_zeros_.size()) == num_non_zeros);

  // Finally, fill in the data for the value and derivatives.
  value_.resize(nr, nc);  // Will do nothing for fixed-size matrix.
  derivatives_.resize(num_non_zeros);  // heap allocation
  for (int n = 0; n < num_non_zeros; ++n) derivatives_[n].resize(nr, nc);
  for (int i = 0; i < sz; ++i) {
    const AutoDiffXd& entry = ad(i);
    const Eigen::VectorXd& derivs = entry.derivatives();
    value_(i) = entry.value();
    if (derivs.size() == 0) {
      for (int n = 0; n < num_non_zeros; ++n) derivatives_[n](i) = 0.0;
      continue;
    }
    DRAKE_ASSERT(static_cast<int>(derivs.size()) == num_variables_);
    for (int n = 0; n < num_non_zeros; ++n)
      derivatives_[n](i) = derivs(non_zeros_[n]);
  }
}

template <typename MatrixT>
DifferentiableMatrix<MatrixT, AutoDiffXd>&
DifferentiableMatrix<MatrixT, AutoDiffXd>::operator=(
    const AutoDiffMatrixType& ad) {
  // Invoke move assignment.
  return *this = DifferentiableMatrix<MatrixT, AutoDiffXd>(ad);
}

template <typename MatrixT>
auto DifferentiableMatrix<MatrixT, AutoDiffXd>::ExtractGradient() const
    -> GradientMatrixType {
  const int sz = value_.size();
  GradientMatrixType gradient = GradientMatrixType::Zero(sz, num_variables_);

  for (int n = 0; n < num_non_zeros(); ++n) {
    const int v = non_zero(n);
    const MatrixType& deriv = derivative(n);
    for (int i = 0; i < sz; ++i) gradient(i, v) = deriv(i);
  }

  return gradient;
}

template <typename MatrixT>
auto DifferentiableMatrix<MatrixT, AutoDiffXd>::ToAutoDiffXd() const
    -> AutoDiffMatrixType {
  const int nr = value_.rows();
  const int nc = value_.cols();
  const int sz = nr * nc;
  AutoDiffMatrixType ad(nr, nc);
  for (int i = 0; i < sz; ++i) {
    ad(i).value() = value_(i);
    ad(i).derivatives() = Eigen::VectorXd::Zero(num_variables_);
  }

  for (int n = 0; n < static_cast<int>(non_zeros_.size()); ++n) {
    const MatrixType& deriv = derivatives_[n];
    for (int i = 0; i < sz; ++i) ad(i).derivatives()(non_zeros_[n]) = deriv(i);
  }

  return ad;
}

template <typename MatrixT>
auto DifferentiableMatrix<MatrixT, AutoDiffXd>::transpose() const -> TransposeType {
  TransposeType matrix_transpose(num_variables(), non_zeros_);
  matrix_transpose.SetValue(value_.transpose());
  for (int n = 0; n < num_non_zeros(); ++n)
    matrix_transpose.SetNonzeroDerivative(n, derivative(n).transpose());
  return matrix_transpose;
}

template <typename MatrixT>
template <typename RhsMatrixType>
auto DifferentiableMatrix<MatrixT, AutoDiffXd>::operator*(
    const DifferentiableMatrix<RhsMatrixType, AutoDiffXd>& rhs) const
    -> MultiplyResultType<RhsMatrixType> {
  DRAKE_DEMAND(num_variables() == rhs.num_variables());

  const bool shapes_are_conforming = DifferentiableMatrixMultiply<
      MatrixType, RhsMatrixType>::ShapesAreConforming(value(), rhs.value());
  DRAKE_DEMAND(shapes_are_conforming);

  // Note that this assumes the non_zero arrays are sorted and unique.
  // We are also assuming both lhs and rhs values are non-zero, so we get
  // a non-zero derivative if either of the source derivatives is non-zero.
  std::vector<int> merged_non_zeros;
  // We'll need at least this much storage but might need more.
  merged_non_zeros.reserve(std::max(num_non_zeros(), rhs.num_non_zeros()));
  std::set_union(non_zeros_.begin(), non_zeros_.end(), rhs.non_zeros_.begin(),
                 rhs.non_zeros_.end(), std::back_inserter(merged_non_zeros));

  MultiplyResultType<RhsMatrixType> result(num_variables_,
                                           std::move(merged_non_zeros));
  result.value_ = value() * rhs.value();

  // ∂a⋅b       ∂ b     ∂ a
  // ---- = a ⋅ ---  +  --- ⋅ b
  //  ∂xᵥ       ∂xᵥ     ∂xᵥ

  int i = 0, j = 0, k = 0;  // non_zero indices for lhs, rhs, result
  for (; i < num_non_zeros() && j < rhs.num_non_zeros(); ++k) {
    const int nzi = non_zero(i);
    const int nzj = rhs.non_zero(j);
    if (nzi == nzj) {
      result.derivatives_[k] =
          value() * rhs.derivative(j++) + derivative(i++) * rhs.value();
    } else if (nzi < nzj) {
      result.derivatives_[k] = derivative(i++) * rhs.value();
    } else {  // nzi > nzj
      result.derivatives_[k] = value() * rhs.derivatives_[j++];
    }
  }
  // Only one of these two loops will execute.
  for (; i < num_non_zeros(); ++i, ++k)
    result.derivatives_[k] = derivative(i) * rhs.value();
  for (; j < rhs.num_non_zeros(); ++j, ++k)
    result.derivatives_[k] = value() * rhs.derivative(j);

  DRAKE_DEMAND(k == result.num_non_zeros());
  return result;
}

}  // namespace math
}  // namespace drake
