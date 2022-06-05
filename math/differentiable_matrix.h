/** @file
A matrix and its derivatives. */

#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
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
For other scalar types T (double or Expression),
`%DifferentiableMatrix<Matrix<T>>` is-a `Matrix<T>` (via CRTP) with
construction and functionality passed through. A few additional methods and
declarations are added to facilitate performant calling code that can use
`%DifferentiableMatrix<Matrix<T>>` without having to special case based on T.

See the @ref differentiable_matrix_specialization "specialization" for details
on the useful features of this class. */
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

/** @anchor differentiable_matrix_specialization

Specialization of DifferentiableMatrix for `Matrix<AutoDiffXd>`.

Let M be the Matrix value, and let `x ≜ {xᵥ : v ∈ [0,nᵥ)}` be the nᵥ variables
with respect to which we are taking partial derivatives.

As an example, assume M is 2x3, nᵥ is 5, and the partials with respect to
x₁ and x₂ are zero. Then in this specialization we store M and the non-zero
partials like this:

    num_variables=5        non_zeros=[0, 3, 4]

    +----------+    +----------+ +----------+ +----------+
    |          |    |          | |          | |          |
    |          |    |          | |          | |          |
    +----------+    +----------+ +----------+ +----------+
         M             ∂M/∂x₀       ∂M/∂x₃       ∂M/∂x₄

The derivatives are identical in structure to the value and they are stored
in consecutive memory locations (in an std::vector).

We can perform computations involving this object and any other
%DifferentiableMatrix or `Matrix<AutoDiffXd>` that has the same nᵥ. As a
special case an operand for which nᵥ=0 (with no non-zero derivatives) is
compatible with any operand regardless of that operand's nᵥ. A constant vector,
for example, is best represented with nᵥ=0.
*/
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

  /** What a row of the input matrix looks like. */
  using AutoDiffRowType = Eigen::Matrix<
      AutoDiffXd, 1, MatrixT::ColsAtCompileTime,
      Eigen::RowMajor, 1, MatrixT::MaxColsAtCompileTime>;

  /** What a column of the input matrix looks like. */
  using AutoDiffColType = Eigen::Matrix<
      AutoDiffXd, MatrixT::RowsAtCompileTime, 1,
      0, MatrixT::MaxRowsAtCompileTime, 1>;

  using AutoDiffMatrixTransposeType = Eigen::Matrix<
      AutoDiffXd, MatrixT::ColsAtCompileTime, MatrixT::RowsAtCompileTime,
      0, MatrixT::MaxColsAtCompileTime, MatrixT::MaxRowsAtCompileTime>;

  /** Constructs with an uninitialized matrix and no derivatives. */
  DifferentiableMatrix() {}

  /** Constructs a %DifferentiableMatrix from a given `Matrix<AutoDiffXd>`.
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
  equivalent of this %DifferentiableMatrix. For this T=AutoDiffXd case, we must
  perform an expensive conversion operation. */
  using EigenEquivalentType = AutoDiffMatrixType;
  using EigenRowType = AutoDiffRowType;
  using EigenColType = AutoDiffColType;

  /** Generates a Matrix<AutoDiffXd> equivalent to this %DifferentialMatrix.
  This is expensive for this specialization and should be avoided. */
  EigenEquivalentType GetEigenEquivalent() const {
    return ToAutoDiffXd();
  }

  /** Generates a RowVector<AutoDiffXd> equivalent to a row of this
  %DifferentiableMatrix. */
  EigenRowType GetEigenRow(int i) const {
    // TODO(sherm1) Create just the row.
    EigenEquivalentType whole_matrix = GetEigenEquivalent();
    return whole_matrix.row(i);
  }

  /** Generates a (column) Vector<AutoDiffXd> equivalent to a column of this
  %DifferentiableMatrix. */
  EigenColType GetEigenCol(int j) const {
    // TODO(sherm1) Create just the column.
    EigenEquivalentType whole_matrix = GetEigenEquivalent();
    return whole_matrix.col(j);
  }

  /** Returns the nᵗʰ non-zero derivative. The corresponding variable is
  xᵥ, where v = non_zero(n). That is, we're returning dₙ = ∂M/∂xᵥ, where M
  is the matrix result returned by value(). */
  const MatrixType& non_zero_derivative(int n) const {
    DRAKE_ASSERT(0 <= n && n < static_cast<int>(non_zero_derivatives_.size()));
    return non_zero_derivatives_[n];
  }

  /** Returns the variable number v for the nᵗʰ non-zero derivative. That is,
  the nᵗʰ non-zero derivative dₙ of this matrix M is dₙ = ∂M/∂xᵥ. */
  int non_zero(int n) const {
    DRAKE_ASSERT(0 <= n && n < num_non_zeros());
    return non_zeros_[n];
  }

  /** Returns the number of partial derivatives whose values are stored
  explicitly, rather than being implicitly considered zero. */
  int num_non_zeros() const { return static_cast<int>(non_zeros_.size()); }

  /** Returns the total number of variables with respect to which we are taking
  derivatives, including those explicitly stored and implicitly zero. If the
  number of variables is returned zero, that means we don't know how may
  variables there are but regardless of that, all the derivatives are known to
  be zero. That representation is best for a known-constant matrix. */
  int num_variables() const { return num_variables_; }

  /** The type produced by the transpose() method. */
  using TransposeType = DifferentiableMatrix<AutoDiffMatrixTransposeType>;

  /** The %DifferentiableMatrix result from multiplying `this`
  matrix times some other %DifferentiableMatrix. */
  template <typename RhsMatrixType>
  using MultiplyResultType =
      DifferentiableMatrix<typename DifferentiableMatrixMultiply<
          MatrixType, RhsMatrixType>::AutoDiffMatrixResultType>;

  /** This is the Eigen `Matrix<AutoDiffXd>` result from multiplying `this`
  matrix times an Eigen `Matrix<AutoDiffXd>`. */
  template <typename RhsMatrixType>
  using EigenMultiplyResultType = typename DifferentiableMatrixMultiply<
      MatrixType, RhsMatrixType>::AutoDiffMatrixResultType;

  using GradientMatrixType = Eigen::Matrix<
      double, MatrixType::SizeAtCompileTime, Eigen::Dynamic>;

  /** Returns a gradient matrix in AutoDiff style -- one row per matrix
  element e, with column(v)=∂e/∂xᵥ. */
  GradientMatrixType ExtractGradient() const;

  /** Returns a `Matrix<AutoDiffXd>` object numerically equivalent to this
  DifferentiableMatrix object. */
  AutoDiffMatrixType ToAutoDiffXd() const;

  /** Returns a %DifferentiableMatrix whose value and derivatives are the
  transpose of this object. */
  TransposeType transpose() const;

  /** Multiplies two %DifferentiableMatrix objects and returns a new one that
  contains the product and its derivatives. The arguments must be
  conformable. */
  template <typename RhsMatrixType>
  MultiplyResultType<RhsMatrixType> operator*(
      const DifferentiableMatrix<RhsMatrixType, AutoDiffXd>& rhs) const;

  /** Multiplies this %DifferentiableMatrix by an Eigen `Matrix<AutoDiffXd>`
  and returns an Eigen `Matrix<AutoDiffXd>` result that contains the product
  and its derivatives. The arguments must be conformable. */
  template <int rows, int cols, int options, int max_rows, int max_cols>
  EigenMultiplyResultType<Eigen::Matrix<AutoDiffXd, rows, cols, Eigen::ColMajor,
                                        max_rows, max_cols>>
  operator*(const Eigen::Matrix<AutoDiffXd, rows, cols, options, max_rows,
                                max_cols>& rhs) const;

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
    non_zero_derivatives_.resize(non_zeros_.size());
  }

  void SetValue(MatrixType&& source) {
    value_ = std::move(source);
  }

  void SetNonzeroDerivative(int n, MatrixType&& source) {
    DRAKE_DEMAND(0 <= n && n < static_cast<int>(non_zeros_.size()));
    non_zero_derivatives_[n] = std::move(source);
  }

  MatrixType value_;
  int num_variables_{0};
  // These two vectors are the same size.
  std::vector<int> non_zeros_;  // Strictly increasing order.
  std::vector<MatrixType> non_zero_derivatives_;
};

/** Returns a reference to the value contained in the given
DifferentiableMatrix. This works for any scalar type and is always fast. */
template <typename MatrixT>
const typename DifferentiableMatrix<MatrixT>::MatrixType&
ExtractValue(const DifferentiableMatrix<MatrixT>& matrix) {
  return matrix.value();
}

/** Returns a gradient matrix in AutoDiff-compatible format that is numerically
equivalent to the derivatives stored in the given DifferentiableMatrix. This is
an expensive conversion operation; you can access the derivatives directly via
the `non_zero_derivative()` method of DifferentiableMatrix.
@see DifferentiableMatrix::non_zero_derivative() */
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
      o << "\n" << matrix.non_zero_derivative(nz_nxt++) << "\n";
  }
  return o;
}

// Some helper functions for the implementations below.
namespace internal {

// Determines how many variables are handled by a given AutoDiffXd Matrix.
// Derivative vectors in ad_matrix are either zero length or all the same
// length nv=number of variables. So we only need to find the first
// derivative vector that has a non-zero length. Zero-length derivatives
// are rare, so we have a good shot at just looking at one entry here.
// Note that we are not checking here for a badly-formed AutoDiffXd Matrix
// in which non-zero derivative vectors aren't all the same length.
template <int rows, int cols, int options, int max_rows, int max_cols>
int FindNumAutoDiffXdVariables(
    const Eigen::Matrix<AutoDiffXd, rows, cols, options, max_rows, max_cols>&
        ad_matrix) {
  const int size = ad_matrix.size();

  int num_variables = 0;
  for (int i = 0; i < size; ++i) {
    const AutoDiffXd& entry = ad_matrix(i);
    // This is an assignment, not an equality test!
    if ((num_variables = entry.derivatives().size()) > 0) break;
  }
  return num_variables;
}

}  // namespace internal

// Definitions of DifferentiableMatrix class methods must be in the header
// since the class is arbitrarily templatized.

template <typename MatrixT>
DifferentiableMatrix<MatrixT, AutoDiffXd>::DifferentiableMatrix(
    const AutoDiffMatrixType& ad) {
  const int nr = ad.rows();
  const int nc = ad.cols();
  const int sz = nr * nc;

  num_variables_ = internal::FindNumAutoDiffXdVariables(ad);

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
  non_zero_derivatives_.resize(num_non_zeros);  // heap allocation
  for (int n = 0; n < num_non_zeros; ++n)
    non_zero_derivatives_[n].resize(nr, nc);
  for (int i = 0; i < sz; ++i) {
    const AutoDiffXd& entry = ad(i);
    const Eigen::VectorXd& derivs = entry.derivatives();
    value_(i) = entry.value();
    if (derivs.size() == 0) {
      for (int n = 0; n < num_non_zeros; ++n) non_zero_derivatives_[n](i) = 0.0;
      continue;
    }
    DRAKE_ASSERT(static_cast<int>(derivs.size()) == num_variables_);
    for (int n = 0; n < num_non_zeros; ++n)
      non_zero_derivatives_[n](i) = derivs(non_zeros_[n]);
  }
}

template <typename MatrixT>
DifferentiableMatrix<MatrixT, AutoDiffXd>&
DifferentiableMatrix<MatrixT, AutoDiffXd>::operator=(
    const AutoDiffMatrixType& ad) {
  // Invoke move assignment after expensive conversion.
  return *this = DifferentiableMatrix<MatrixT, AutoDiffXd>(ad);
}

template <typename MatrixT>
auto DifferentiableMatrix<MatrixT, AutoDiffXd>::ExtractGradient() const
    -> GradientMatrixType {
  const int sz = value_.size();
  GradientMatrixType gradient = GradientMatrixType::Zero(sz, num_variables_);

  for (int n = 0; n < num_non_zeros(); ++n) {
    const int v = non_zero(n);
    const MatrixType& deriv = non_zero_derivative(n);
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
    const MatrixType& deriv = non_zero_derivatives_[n];
    for (int i = 0; i < sz; ++i) ad(i).derivatives()(non_zeros_[n]) = deriv(i);
  }

  return ad;
}

template <typename MatrixT>
auto DifferentiableMatrix<MatrixT, AutoDiffXd>::transpose() const
    -> TransposeType {
  TransposeType matrix_transpose(num_variables(), non_zeros_);
  matrix_transpose.SetValue(value_.transpose());
  for (int n = 0; n < num_non_zeros(); ++n)
    matrix_transpose.SetNonzeroDerivative(n,
                                          non_zero_derivative(n).transpose());
  return matrix_transpose;
}

template <typename MatrixT>
template <typename RhsMatrixType>
auto DifferentiableMatrix<MatrixT, AutoDiffXd>::operator*(
    const DifferentiableMatrix<RhsMatrixType, AutoDiffXd>& rhs) const
    -> MultiplyResultType<RhsMatrixType> {
  // The number of variables in both operands must match or be zero.
  const int result_num_variables = [&]() {
    if (num_variables() == 0) {
      DRAKE_ASSERT(num_non_zeros() == 0);
      return rhs.num_variables();
    }
    if (rhs.num_variables() == 0) {
      DRAKE_ASSERT(rhs.num_non_zeros() == 0);
      return num_variables();
    }
    DRAKE_DEMAND(num_variables() == rhs.num_variables());
    return num_variables();
  }();

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

  MultiplyResultType<RhsMatrixType> result(result_num_variables,
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
      result.non_zero_derivatives_[k] = value() * rhs.non_zero_derivative(j++) +
                                        non_zero_derivative(i++) * rhs.value();
    } else if (nzi < nzj) {
      result.non_zero_derivatives_[k] = non_zero_derivative(i++) * rhs.value();
    } else {  // nzj < nzi
      result.non_zero_derivatives_[k] =
          value() * rhs.non_zero_derivatives_[j++];
    }
  }
  // At most one of these two loops will execute.
  for (; i < num_non_zeros(); ++i, ++k)
    result.non_zero_derivatives_[k] = non_zero_derivative(i) * rhs.value();
  for (; j < rhs.num_non_zeros(); ++j, ++k)
    result.non_zero_derivatives_[k] = value() * rhs.non_zero_derivative(j);

  DRAKE_DEMAND(k == result.num_non_zeros());
  return result;
}

// This is equivalent to
//   result = this->ToAutoDiffXd() * rhs
// but works directly with the DifferentiableMatrix and AutoDiff matrices to
// avoid the expensive conversions.
template <typename MatrixT>
template <int rows, int cols, int options, int max_rows, int max_cols>
auto DifferentiableMatrix<MatrixT, AutoDiffXd>::operator*(
    const Eigen::Matrix<AutoDiffXd, rows, cols, options, max_rows, max_cols>&
        rhs) const
    -> EigenMultiplyResultType<Eigen::Matrix<
        AutoDiffXd, rows, cols, Eigen::ColMajor, max_rows, max_cols>> {
  // The result of m×p ⋅ p×n is m×n. Initially each AutoDiffXd element has an
  // empty derivatives array.
  Eigen::Matrix<AutoDiffXd, MatrixT::RowsAtCompileTime, cols, Eigen::ColMajor,
                MatrixT::MaxRowsAtCompileTime, max_cols>
      result;

  // Either or both matrices may be dynamically sized so we might need to
  // dynamically size the result as well. This does nothing if both rows and
  // columns are fixed sizes.
  result.resize(value().rows(), rhs.cols());

  // The result will have the same number of variables as both inputs do. We'll
  // get that number as cheaply as possible here and defer checking for
  // compatibility until we're already touching the entries. (Zero is compatible
  // with anything -- we just assume all-zero derivatives as needed.)
  int assumed_num_variables = num_variables();
  if (assumed_num_variables == 0) {
    assumed_num_variables = internal::FindNumAutoDiffXdVariables(rhs);
  }

  // ∂a⋅b       ∂ b     ∂ a
  // ---- = a ⋅ ---  +  --- ⋅ b
  //  ∂xᵥ       ∂xᵥ     ∂xᵥ

  // Strategy here will be to work on each result element once. Element i,j
  // looks like:
  //   Rij | ∂Rij/∂x₀ ∂Rij/∂x₁ ⋅⋅⋅ ∂Rij/∂xₙᵥ₋₁
  // Where
  //   Rij = a[i] ⋅ b(j)     where [] picks a row and () picks a column
  //   ∂Rij/∂xᵥ = a[i] ⋅ ∂b(j)/∂xᵥ + ∂a[i]/∂xᵥ ⋅ b(j)

  const auto& a = value();  // The DifferentiableMatrix value as Matrix<double>.
  const auto& b = rhs;      // The Matrix<AutoDiffXd>.

  // Compute the result in column storage order (row changes fastest).
  for (int j=0; j < result.cols(); ++j) {
    for (int i=0; i < result.rows(); ++i) {
      // The Eigen indexing operators applied to Maps return copies (see Eigen
      // issue #2076 on GitLab). Use coeffRef() to make sure we get a real
      // reference. (A Block is a Map)
      const auto ai = a.row(i);  // Eigen::Block, be careful!
      const auto bj = b.col(j);

      // This is the element we're going to compute now.
      AutoDiffXd& rij = result(i, j);

      rij.value() = 0;  // Accumulate element value.
      for (int k=0; k < a.cols(); ++k) {  // rij = ai ⋅ bj
        const double& aik = ai.coeffRef(k);
        const double& bjk = bj.coeffRef(k).value();
        rij.value() += aik * bjk;
      }

      if (assumed_num_variables == 0)
        continue;  // We need only do the value; no derivatives.

      // drij hasn't necessarily been allocated yet.
      Eigen::VectorXd& drij = rij.derivatives();  // ∂rij/∂x
      // TODO(sherm1) Defer this allocation.
      drij = Eigen::VectorXd::Zero(assumed_num_variables);

      // a[i] ⋅ ∂b(j)/∂xᵥ
      for (int k=0; k < a.cols(); ++k) {
        const double& aik = ai.coeffRef(k);
        const AutoDiffXd& bjk = bj.coeffRef(k);
        const Eigen::VectorXd& dbjk = bjk.derivatives();
        if (dbjk.size() > 0) {
          DRAKE_DEMAND(dbjk.size() == assumed_num_variables);
          for (int v = 0; v < assumed_num_variables; ++v) {
            drij[v] += aik * dbjk[v];
          }
        }
      }

      // ∂a[i]/∂xᵥ ⋅ b(j)
      for (int n=0; n < num_non_zeros(); ++n) {
        const int v = non_zero(n);
        const auto& dadv = non_zero_derivative(n);
        DRAKE_ASSERT(dadv.rows() == a.rows() && dadv.cols() == a.cols());
        const auto dadv_ri = dadv.row(i);  // Eigen::Block
        double& drijv = drij[v];
        for (int k=0; k < a.cols(); ++k) {
          const double& dadv_rik = dadv_ri.coeffRef(k);
          const double& bjk = bj.coeffRef(k).value();
          drijv += dadv_rik * bjk;
        }
      }
    }
  }

  return result;
}

}  // namespace math
}  // namespace drake
