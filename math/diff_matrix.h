/** @file
A matrix and its derivatives. */

#pragma once

#include <algorithm>
#include <cmath>
#include <ostream>
#include <type_traits>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/autodiff.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic/expression.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace math {

/** An efficient data structure for holding a matrix and its partial
derivatives. Template parameters follow Eigen's as closely as possible but
operator support is limited. This is intended as a more-efficient alternative
to using Eigen::Matrix<AutoDiffXd> and uses only `double` scalars.

Let M be the matrix value, and let `v ≜ {vₖ : k ∈ [0,nᵥ)}` be the nᵥ variables
with respect to which we are taking partial derivatives. As an example, assume
M is 2x3, nᵥ is 5, and the partials with respect to v₁ and v₂ are zero. Then we
store M and the non-zero partials like this:

    num_variables=5        non_zeros=[0, 3, 4]

    +----------+    +----------+ +----------+ +----------+
    |          |    |          | |          | |          |
    |          |    |          | |          | |          |
    +----------+    +----------+ +----------+ +----------+
         M             ∂M/∂v₀       ∂M/∂v₃       ∂M/∂v₄

The derivatives are identical in structure to the value and they are stored
in consecutive memory locations.

We can perform computations involving this object and any other
%DiffMatrix or `Matrix<AutoDiffXd>` that has the same nᵥ. As a
special case an operand for which nᵥ=0 (with no non-zero derivatives) is
compatible with any operand regardless of that operand's nᵥ. A constant matrix,
for example, is best represented with nᵥ=0.

As an example of where this can be particularly performant, consider
differentiating a matrix inverse (assuming M is invertible). Rather than
differentiating the inversion _algorithm_ (element by AutoDiffXd element), we
need only compute the inverse M⁻¹ efficiently in double and then apply the
identity `∂M⁻¹/∂v = −M⁻¹ ∂M/∂v M⁻¹` to each of the non-zero derivative
matrices using Eigen's ordinary matrix multiply.

### Usage

In general we write code that can work with any of the standard Drake scalar
types T, where T==double, symbolic::Expression, or AutoDiffXd. For best
performance we want to use ordinary Eigen matrices when T is double or
Expression, but a DiffMatrix when T is AutoDiffXd. We provide a templatized
typedef `MaybeDiffMatrix<T, ...>` for this purpose, where `...` are the usual
Eigen matrix parameters. Common operators are provided in DiffMatrix so that
most calling code can work without specialization, and a compile time bool
`is_diff_matrix<MatrixType>` is provided for cases where specialization is
needed. Example:
<pre>
  using MyMatrixType = MaybeDiffMatrix<T, 3, 3>;
  ...
  if constexpr (is_diff_matrix<MyMatrixType>)
    // code specialized for a DiffMatrix
  else
    // code specialized for an ordinary Eigen matrix
</pre>
*/
template <int _rows, int _cols, int _options = 0, int _max_rows = _rows,
          int _max_cols = _cols>
class DiffMatrix {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DiffMatrix);

  // Names chosen to follow Eigen conventions. (Better not to use an enum
  // here to avoid warnings when comparing against these values in other
  // matrix types.)
  static constexpr int RowsAtCompileTime = _rows;
  static constexpr int ColsAtCompileTime = _cols;
  static constexpr int Options = _options;
  static constexpr int MaxRowsAtCompileTime = _max_rows;
  static constexpr int MaxColsAtCompileTime = _max_cols;
  static constexpr int IsRowMajor = _options & Eigen::RowMajor;
  static constexpr int IsDynamic =
      _rows == Eigen::Dynamic || _cols == Eigen::Dynamic;
  static constexpr int IsDiffMatrix = true;

  /** The type of the `Matrix<double>` we'll use to store the value and
  derivatives. */
  using MatrixType =
      Eigen::Matrix<double, RowsAtCompileTime, ColsAtCompileTime, Options,
                    MaxRowsAtCompileTime, MaxColsAtCompileTime>;

  /** The `Matrix<AutoDiffXd>` equivalent to this %DiffMatrix. We
  need to be able to convert from and to this format. */
  using AutoDiffMatrixType = MatrixLikewise<AutoDiffXd, MatrixType>;

  /** The type returned by the transpose() and inverse() methods. Note that the
  storage order is column major regardless of `this` storage order. */
  using TransposeType =
      DiffMatrix<ColsAtCompileTime, RowsAtCompileTime, Eigen::ColMajor,
                 MaxColsAtCompileTime, MaxRowsAtCompileTime>;

  /** What's in the value() of a TransposeType. */
  using TransposeMatrixType =
      Eigen::Matrix<double, ColsAtCompileTime, RowsAtCompileTime,
                    Eigen::ColMajor, MaxColsAtCompileTime,
                    MaxRowsAtCompileTime>;

  // Inverse and Transpose have the same shape, although this will be a
  // pseudo-inverse if it isn't square.
  using InverseType = TransposeType;
  using InverseMatrixType = TransposeMatrixType;

  /** If we have to extract an Eigen-equivalent row-of-AutoDiffXds, this is
  the return type. */
  using AutoDiffRowType =
      Eigen::Matrix<AutoDiffXd, 1, ColsAtCompileTime, Eigen::RowMajor, 1,
                    MaxColsAtCompileTime>;

  /** If we have to extract an Eigen-equivalent column-of-AutoDiffXds, this is
  the return type. */
  using AutoDiffColType =
      Eigen::Matrix<AutoDiffXd, RowsAtCompileTime, 1, Eigen::ColMajor,
                    MaxRowsAtCompileTime, 1>;

  /** Constructs with an uninitialized matrix and no derivatives. */
  DiffMatrix() {}

  /** Constructs a %DiffMatrix from a given `Matrix<AutoDiffXd>`.
  The input matrix template parameters must match those for the
  DiffMatrix. */
  explicit DiffMatrix(const AutoDiffMatrixType& ad);

  /** Sets this %DiffMatrix from a given `Matrix<AutoDiffXd>`.
  The input matrix template parameters must match those for the
  DiffMatrix. */
  DiffMatrix& operator=(const AutoDiffMatrixType& ad);

  /** Returns the stored value with scalars of type double. */
  const MatrixType& value() const { return value_; }

  /** Generates a RowVector<AutoDiffXd> equivalent to a row of this
  %DiffMatrix. */
  AutoDiffRowType GetEigenRow(int i) const {
    // TODO(sherm1) Create just the row.
    AutoDiffMatrixType whole_matrix = ToAutoDiffXd();
    return whole_matrix.row(i);
  }

  /** Generates a (column) Vector<AutoDiffXd> equivalent to a column of this
  %DiffMatrix. */
  AutoDiffColType GetEigenCol(int j) const {
    // TODO(sherm1) Create just the column.
    AutoDiffMatrixType whole_matrix = ToAutoDiffXd();
    return whole_matrix.col(j);
  }

  /** Returns the nᵗʰ non-zero derivative. The corresponding variable is
  vₖ, where k = non_zero(n). That is, we're returning dₙ = ∂M/∂vₖ, where M
  is the matrix result returned by value(). */
  const MatrixType& non_zero_derivative(int n) const {
    DRAKE_ASSERT(0 <= n && n < static_cast<int>(non_zero_derivatives_.size()));
    return non_zero_derivatives_[n];
  }

  /** Returns the variable number k for the nᵗʰ non-zero derivative. That is,
  the nᵗʰ non-zero derivative dₙ of this matrix M is dₙ = ∂M/∂vₖ. */
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

  /** The type of the %DiffMatrix result from multiplying `this`
  matrix on the left times some other %DiffMatrix on the right. The
  result is in column major storage regardless of the operands' order.  */
  template <typename RhsMatrixType>
  using MultiplyResultType =
      DiffMatrix<RowsAtCompileTime, RhsMatrixType::ColsAtCompileTime,
                           Eigen::ColMajor, MaxRowsAtCompileTime,
                           RhsMatrixType::MaxColsAtCompileTime>;

  /** Type type of the Eigen `Matrix<AutoDiffXd>` result from multiplying `this`
  matrix on the left times an Eigen `Matrix<AutoDiffXd>` on the right. The
  result is in column major storage regardless of the operands' order. */
  template <typename RhsMatrixType>
  using EigenMultiplyResultType =
      Eigen::Matrix<AutoDiffXd, RowsAtCompileTime,
                    RhsMatrixType::ColsAtCompileTime, Eigen::ColMajor,
                    MaxRowsAtCompileTime, RhsMatrixType::MaxColsAtCompileTime>;

  using GradientMatrixType = Eigen::Matrix<
      double, MatrixType::SizeAtCompileTime, Eigen::Dynamic>;

  /** Returns a gradient matrix in AutoDiff style -- one row per matrix
  element eᵢⱼ, with row entry k=∂eᵢⱼ/∂vₖ. */
  GradientMatrixType ExtractGradient() const;

  /** Returns a `Matrix<AutoDiffXd>` object numerically equivalent to this
  DiffMatrix object. */
  AutoDiffMatrixType ToAutoDiffXd() const;

  /** Returns a %DiffMatrix whose value and derivatives are the
  transpose of this object. */
  TransposeType transpose() const;

  /** Returns a %DiffMatrix whose value is the inverse of `this`
  and whose derivatives are the derivatives of the inverse. The matrix value
  must be square and invertible using Eigen's matrix.inverse() method. */
  InverseType inverse() const;

  /** Given a suitably-calculated double inverse of this %DifferentialMatrix
  value, use it to create a %DiffMatrix representing the inverse
  and its derivatives. The supplied inverse must be the appropriate shape,
  that is, a transpose of the value dimensions. The returned
  %DiffMatrix has as its value the evaluated inverse and its
  derivatives are calculated using the identity `∂M⁻¹/∂v = −M⁻¹ ∂M/∂v M⁻¹`. */
  template <typename Derived>
  InverseType inverse(const Eigen::MatrixBase<Derived>& inv_M) const;

  /** Given an inverse matrix that exactly matches the internally
  stored InverseMatrixType in the result, move from it if possible. */
  InverseType inverse(InverseMatrixType&& inv_M) const;

  /** Returns a %DiffMatrix whose value is the pseudoinverse M⁺ₙₓₘ
  of `this` matrix Mₘₓₙ and whose derivatives are the derivatives ∂M⁺ of the
  pseudoinverse. The value matrix M can be any shape and rank-deficient. We use
  Eigen's CompleteOrthogonalDecomposition::pseudoInverse() method to perform the
  computation, and its rank() method (using the default threshold) to guide the
  derivative calculation.

  There are three special cases if Mₘₓₙ has "full" rank min(m,n). In that case
  the pseudoinverse is one of

      (1) M⁺ = (MᵀM)⁻¹Mᵀ if m > n (left pseudoinverse; M⁺M=Iₙ), or
      (2) M⁺ = Mᵀ(MMᵀ)⁻¹ if m < n (right pseudoinverse; MM⁺=Iₘ).
      (3) M⁺ = M⁻¹ if m==n (ordinary inverse)

  If M has rank < min(m,n) the pseudoinverse still exists but must be
  calculated by other means such as singular value decomposition or complete
  orthogonal decomposition. In the general case we have

      ∂M⁺ = [(M⁺M−Iₙ) ∂Mᵀ M⁺ᵀM⁺] + [M⁺M⁺ᵀ ∂Mᵀ (MM⁺−Iₘ)] − [M⁺ ∂M M⁺]

  In special case (1), the first bracketed term is 0. In special case (2),
  the second bracketed term is 0, and in special case (3), only the last
  bracketed term is needed.
  */

  InverseType PseudoInverse() const;

  template <typename Derived>
  InverseType PseudoInverse(const Eigen::MatrixBase<Derived>& pinv_M,
                            int rank) const;

  InverseType PseudoInverse(InverseMatrixType&& pinv_M, int rank) const;

  /** Multiplies two %DiffMatrix objects and returns a new one that
  contains the product and its derivatives. The arguments must be
  conformable. */
  template <int rhs_rows, int rhs_cols, int rhs_options, int rhs_max_rows,
            int rhs_max_cols>
  auto operator*(const DiffMatrix<rhs_rows, rhs_cols, rhs_options,
                                            rhs_max_rows, rhs_max_cols>& rhs)
      const -> MultiplyResultType<DiffMatrix<
          rhs_rows, rhs_cols, rhs_options, rhs_max_rows, rhs_max_cols>>;

  /** Multiplies this %DiffMatrix by an Eigen `Matrix<AutoDiffXd>`
  and returns an Eigen `Matrix<AutoDiffXd>` result that contains the product
  and its derivatives. The arguments must be conformable. */
  template <int rhs_rows, int rhs_cols, int rhs_options, int rhs_max_rows,
            int rhs_max_cols>
  auto operator*(
      const Eigen::Matrix<AutoDiffXd, rhs_rows, rhs_cols, rhs_options,
                          rhs_max_rows, rhs_max_cols>& rhs) const
      -> EigenMultiplyResultType<
          Eigen::Matrix<AutoDiffXd, rhs_rows, rhs_cols, rhs_options,
                        rhs_max_rows, rhs_max_cols>>;

  /** Check whether the given matrix can serve as the rhs of a multiply with
  `this` matrix on the left. (The number of rhs rows must match the number of
  lhs columns.) Resolved at compile time if possible.
  @tparam MaybeRhs any matrix type that can provide Eigen-style
                   RowsAtCompileTime */
  template <typename MaybeRhs>
  bool can_be_multiply_rhs(const MaybeRhs& rhs) const {
    // For fixed-size matrices we can verify conformance at compile time.
    if constexpr (ColsAtCompileTime != Eigen::Dynamic &&
                  MaybeRhs::RowsAtCompileTime != Eigen::Dynamic) {
      return ColsAtCompileTime == MaybeRhs::RowsAtCompileTime;
    } else {  // We need a runtime check.
      return value().cols() == rhs.rows();
    }
  }

  /** Cast to another scalar type. For double we can just return a reference
  to the value. For symbolic::Expression we cast the value. For AutoDiffXd, we
  need to convert. */
  template <typename U>
  decltype(auto) cast() const {  // Return type is inferred.
    static_assert(std::is_same_v<U, double> ||
                  std::is_same_v<U, symbolic::Expression> ||
                  std::is_same_v<U, AutoDiffXd>);
    if constexpr (std::is_same_v<U, double>) return value();
    if constexpr (std::is_same_v<U, symbolic::Expression>) {
      // Eigen's eval() is insufficient here to get the type we want.
      return static_cast<MatrixLikewise<symbolic::Expression, MatrixType>>(
          value().template cast<symbolic::Expression>());
    }
    if constexpr (std::is_same_v<U, AutoDiffXd>) return ToAutoDiffXd();
  }

 private:
  friend TransposeType;
  template <int, int, int, int, int>
  friend class DiffMatrix;

  DiffMatrix(int num_variables, const std::vector<int>& non_zeros)
      : DiffMatrix(num_variables, std::vector<int>(non_zeros)) {}

  DiffMatrix(int num_variables, std::vector<int>&& non_zeros)
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

  // Check whether the given matrix can serve as the inverse of `this` matrix,
  // meaning that it's shape is the transpose of `this`.
  template <typename MaybeInverse>
  bool can_be_inverse(const MaybeInverse& rhs) {
    // For fixed-size matrices we can verify compatibility at compile time.
    if constexpr (!IsDynamic &&
                  MaybeInverse::RowsAtCompileTime != Eigen::Dynamic
                  && MaybeInverse::ColsAtCompileTime != Eigen::Dynamic) {
      return MaybeInverse::RowsAtCompileTime == ColsAtCompileTime
      && MaybeInverse::ColsAtCompileTime == RowsAtCompileTime;
    } else {  // We need a runtime check.
      return rhs.rows() == value().cols() && rhs.cols() == value().rows();
    }
  }

  MatrixType value_;
  int num_variables_{0};
  // These two vectors are the same size.
  std::vector<int> non_zeros_;  // Strictly increasing order.
  std::vector<MatrixType> non_zero_derivatives_;
};

/** Returns a no-cost reference to the value contained in the given
DiffMatrix.
@pydrake_mkdoc_identifier{diffmatrix} */
template <int rows, int cols, int options, int max_rows, int max_cols>
decltype(auto) ExtractValue(
    const DiffMatrix<rows, cols, options, max_rows, max_cols>&
        matrix) {
  return matrix.value();
}

/** Returns a gradient matrix in AutoDiff-compatible format that is numerically
equivalent to the derivatives stored in the given %DiffMatrix. This is
an expensive conversion operation; you can access the derivatives directly via
the `non_zero_derivative()` method of %DiffMatrix.
@see DiffMatrix::non_zero_derivative()
@pydrake_mkdoc_identifier{diffmatrix} */
template <int rows, int cols, int options, int max_rows, int max_cols>
decltype(auto) ExtractGradient(
    const DiffMatrix<rows, cols, options, max_rows, max_cols>&
        matrix) {
  return matrix.ExtractGradient();
}

/** Returns a no-cost reference to the value contained in the given
DiffMatrix. */
template <int rows, int cols, int options, int max_rows, int max_cols>
decltype(auto) DiffExtractValue(
    const DiffMatrix<rows, cols, options, max_rows, max_cols>&
        matrix) {
  return ExtractValue(matrix);  // See above.
}

/** DiffExtractValue() applied to a Matrix<AutoDiffXd> copies out just the
value part into a Matrix<double>. DiffExtractValue() applied to a Matrix<T> for
T != AutoDiffXd just returns the matrix argument at no cost.*/
template <typename Derived>
decltype(auto) DiffExtractValue(
    const Eigen::MatrixBase<Derived>& matrix) {
  if constexpr (std::is_same_v<typename Derived::Scalar, AutoDiffXd>) {
    return ExtractValue(matrix);
  } else {
    return matrix.derived();
  }
}

/** Given a DiffMatrix, converts it at great expense to
Matrix<AutoDiffXd>, but no cost if given an Eigen matrix. */
template <typename T, int rows, int cols, int options, int max_rows,
          int max_cols>
auto MaybeConvertToEigenMatrix(
    const Eigen::Matrix<T, rows, cols, options, max_rows, max_cols>& matrix)
    -> const Eigen::Matrix<T, rows, cols, options, max_rows, max_cols>& {
  return matrix;
}

template <typename T, int rows, int cols, int options, int max_rows,
          int max_cols>
auto MaybeConvertToEigenRow(
    const Eigen::Matrix<T, rows, cols, options, max_rows, max_cols>& matrix,
    int i) {
  return matrix.row(i);
}

template <typename T, int rows, int cols, int options, int max_rows,
          int max_cols>
auto MaybeConvertToEigenCol(
    const Eigen::Matrix<T, rows, cols, options, max_rows, max_cols>& matrix,
    int j) {
  return matrix.col(j);
}

template <int rows, int cols, int options, int max_rows, int max_cols>
auto MaybeConvertToEigenMatrix(
    const DiffMatrix<rows, cols, options, max_rows, max_cols>&
        matrix) {
  return matrix.ToAutoDiffXd();
}

template <int rows, int cols, int options, int max_rows, int max_cols>
auto MaybeConvertToEigenRow(
    const DiffMatrix<rows, cols, options, max_rows, max_cols>&
        matrix, int i) {
  return matrix.GetEigenRow(i);
}

template <int rows, int cols, int options, int max_rows, int max_cols>
auto MaybeConvertToEigenCol(
    const DiffMatrix<rows, cols, options, max_rows, max_cols>&
        matrix, int j) {
  return matrix.GetEigenCol(j);
}

/** Outputs a representation of a DiffMatrix in human-readable form. */
template <int rows, int cols, int options, int max_rows, int max_cols>
std::ostream& operator<<(
    std::ostream& o,
    const DiffMatrix<rows, cols, options, max_rows, max_cols>& matrix) {
  o << "x=\n" << matrix.value() << "\n";
  int nz_nxt = 0;
  for (int i = 0; i < matrix.num_variables(); ++i) {
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

// Determines how many variables (partial derivatives) are tracked by a given
// AutoDiffXd Matrix.
// Derivative vectors in ad_matrix are either zero length or all the same
// length nᵥ=number of variables. So we only need to find the first
// derivative vector that has a non-zero length. Zero-length derivatives
// are rare, so we have a good shot at just looking at one entry here.
// Note that we are not checking here for a badly-formed AutoDiffXd Matrix
// in which non-zero derivative vectors aren't all the same length; that gets
// checked later when we are processing all the entries.
template <int _rows, int _cols, int _options, int _max_rows, int _max_cols>
int FindNumAutoDiffXdVariables(
    const Eigen::Matrix<AutoDiffXd, _rows, _cols, _options, _max_rows,
                        _max_cols>& ad_matrix) {
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

// Definitions of DiffMatrix class methods must be in the header
// since the class is arbitrarily templatized.

template <int _rows, int _cols, int _options, int _max_rows, int _max_cols>
DiffMatrix<_rows, _cols, _options, _max_rows, _max_cols>::DiffMatrix(
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

template <int _rows, int _cols, int _options, int _max_rows, int _max_cols>
auto DiffMatrix<_rows, _cols, _options, _max_rows, _max_cols>::operator=(
    const AutoDiffMatrixType& ad) -> DiffMatrix& {
  // Invoke move assignment after expensive conversion.
  return *this = DiffMatrix(ad);
}

template <int _rows, int _cols, int _options, int _max_rows, int _max_cols>
auto DiffMatrix<_rows, _cols, _options, _max_rows, _max_cols>::ExtractGradient()
    const -> GradientMatrixType {
  const int sz = value_.size();
  GradientMatrixType gradient = GradientMatrixType::Zero(sz, num_variables_);

  for (int n = 0; n < num_non_zeros(); ++n) {
    const int v = non_zero(n);
    const MatrixType& deriv = non_zero_derivative(n);
    for (int i = 0; i < sz; ++i) gradient(i, v) = deriv(i);
  }

  return gradient;
}

template <int _rows, int _cols, int _options, int _max_rows, int _max_cols>
auto DiffMatrix<_rows, _cols, _options, _max_rows, _max_cols>::ToAutoDiffXd()
    const -> AutoDiffMatrixType {
  const int nr = value_.rows();
  const int nc = value_.cols();
  const int sz = nr * nc;
  AutoDiffMatrixType ad(nr, nc);

  // Set the value and initialize all derivatives to zero.
  for (int i = 0; i < sz; ++i) {
    ad(i).value() = value_(i);
    ad(i).derivatives() = Eigen::VectorXd::Zero(num_variables_);
  }

  // Fill in the (possibly) non-zero derivatives.
  for (int n = 0; n < static_cast<int>(non_zeros_.size()); ++n) {
    const int k = non_zeros_[n];                        // vₖ
    const MatrixType& dMdk = non_zero_derivatives_[n];  // ∂M/∂vₖ
    for (int i = 0; i < sz; ++i) ad(i).derivatives()(k) = dMdk(i);
  }

  return ad;
}

template <int _rows, int _cols, int _options, int _max_rows, int _max_cols>
auto DiffMatrix<_rows, _cols, _options, _max_rows, _max_cols>::transpose() const
    -> TransposeType {
  TransposeType matrix_transpose(num_variables(), non_zeros_);
  matrix_transpose.SetValue(value_.transpose());
  for (int n = 0; n < num_non_zeros(); ++n)
    matrix_transpose.SetNonzeroDerivative(n,
                                          non_zero_derivative(n).transpose());
  return matrix_transpose;
}

template <int _rows, int _cols, int _options, int _max_rows, int _max_cols>
template <int rhs_rows, int rhs_cols, int rhs_options, int rhs_max_rows,
          int rhs_max_cols>
auto DiffMatrix<_rows, _cols, _options, _max_rows, _max_cols>::operator*(
    const DiffMatrix<rhs_rows, rhs_cols, rhs_options, rhs_max_rows,
                     rhs_max_cols>& rhs) const
    -> MultiplyResultType<DiffMatrix<rhs_rows, rhs_cols, rhs_options,
                                     rhs_max_rows, rhs_max_cols>> {
  using RhsMatrixType =
      DiffMatrix<rhs_rows, rhs_cols, rhs_options, rhs_max_rows, rhs_max_cols>;

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

  const bool shapes_are_conforming = can_be_multiply_rhs<RhsMatrixType>(rhs);
  DRAKE_DEMAND(shapes_are_conforming);

  // Note that this assumes the non_zero arrays are sorted and unique.
  // We also behave as though lhs and rhs *values* are non-zero, so we get
  // a "non-zero" derivative if either of the source derivatives is non-zero,
  // even though the actual result might still be zero.
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
  //  ∂vₖ       ∂vₖ     ∂vₖ

  int i = 0, j = 0, r = 0;  // non_zero indices for lhs, rhs, result
  for (; i < num_non_zeros() && j < rhs.num_non_zeros(); ++r) {
    const int nzi = non_zero(i);
    const int nzj = rhs.non_zero(j);
    if (nzi == nzj) {
      result.non_zero_derivatives_[r] = non_zero_derivative(i++) * rhs.value() +
                                        value() * rhs.non_zero_derivative(j++);
    } else if (nzi < nzj) {
      result.non_zero_derivatives_[r] = non_zero_derivative(i++) * rhs.value();
    } else {  // nzj < nzi
      result.non_zero_derivatives_[r] = value() * rhs.non_zero_derivative(j++);
    }
  }
  // At most one of these two loops will execute.
  for (; i < num_non_zeros(); ++i, ++r)
    result.non_zero_derivatives_[r] = non_zero_derivative(i) * rhs.value();
  for (; j < rhs.num_non_zeros(); ++j, ++r)
    result.non_zero_derivatives_[r] = value() * rhs.non_zero_derivative(j);

  DRAKE_DEMAND(r == result.num_non_zeros());
  return result;
}

// This is equivalent to
//   result = this->ToAutoDiffXd() * rhs
// but works directly with the DiffMatrix and AutoDiff matrices to
// avoid the expensive conversions.
template <int _rows, int _cols, int _options, int _max_rows, int _max_cols>
template <int rhs_rows, int rhs_cols, int rhs_options, int rhs_max_rows,
          int rhs_max_cols>
auto DiffMatrix<_rows, _cols, _options, _max_rows, _max_cols>::operator*(
    const Eigen::Matrix<AutoDiffXd, rhs_rows, rhs_cols, rhs_options,
                        rhs_max_rows, rhs_max_cols>& rhs) const
    -> EigenMultiplyResultType<
        Eigen::Matrix<AutoDiffXd, rhs_rows, rhs_cols, rhs_options, rhs_max_rows,
                      rhs_max_cols>> {
  // The result of m×p ⋅ p×n is m×n. Initially each AutoDiffXd element has an
  // empty derivatives array.
  EigenMultiplyResultType<Eigen::Matrix<
      AutoDiffXd, rhs_rows, rhs_cols, rhs_options, rhs_max_rows, rhs_max_cols>>
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
  //  ∂vₖ       ∂vₖ     ∂vₖ

  // Strategy here will be to work on each result element once. Element i,j
  // looks like:
  //   Rij | ∂Rij/∂v₀ ∂Rij/∂v₁ ⋅⋅⋅ ∂Rij/∂vₙᵥ₋₁
  // Where
  //   Rij = a[i] ⋅ b(j)     where [] picks a row and () picks a column
  //   ∂Rij/∂vₖ = a[i] ⋅ ∂b(j)/∂vₖ + ∂a[i]/∂vₖ ⋅ b(j)

  const auto& a = value();  // The DiffMatrix value as Matrix<double>.
  const auto& b = rhs;      // The Matrix<AutoDiffXd>.

  // Compute the result in column storage order (row changes fastest).
  for (int j = 0; j < result.cols(); ++j) {
    for (int i = 0; i < result.rows(); ++i) {
      // The Eigen indexing operators applied to Maps return copies (see Eigen
      // issue #2076 on GitLab). Use coeffRef() to make sure we get a real
      // reference. (A Block is a Map)
      const auto ai = a.row(i);  // Eigen::Block, be careful!
      const auto bj = b.col(j);

      // This is the element we're going to compute now.
      AutoDiffXd& rij = result(i, j);

      rij.value() = 0;                      // Accumulate element value.
      for (int p = 0; p < a.cols(); ++p) {  // rij = ai ⋅ bj
        const double& aip = ai.coeffRef(p);
        const double& bjp = bj.coeffRef(p).value();
        rij.value() += aip * bjp;
      }

      if (assumed_num_variables == 0)
        continue;  // We need only do the value; no derivatives.

      // drij hasn't necessarily been allocated yet.
      Eigen::VectorXd& drij = rij.derivatives();  // ∂rij/∂v
      // TODO(sherm1) Defer this allocation until we see a non-zero.
      drij = Eigen::VectorXd::Zero(assumed_num_variables);

      // a[i] ⋅ ∂b(j)/∂vₖ
      for (int p = 0; p < a.cols(); ++p) {
        const double& aip = ai.coeffRef(p);
        const AutoDiffXd& bjp = bj.coeffRef(p);
        const Eigen::VectorXd& dbjp = bjp.derivatives();
        if (dbjp.size() > 0) {
          DRAKE_DEMAND(dbjp.size() == assumed_num_variables);
          for (int k = 0; k < assumed_num_variables; ++k) {
            drij[k] += aip * dbjp[k];
          }
        }
      }

      // ∂a[i]/∂vₖ ⋅ b(j)
      for (int n = 0; n < num_non_zeros(); ++n) {
        const int k = non_zero(n);
        const auto& dadk = non_zero_derivative(n);  // ∂a[i]/∂vₖ
        DRAKE_ASSERT(dadk.rows() == a.rows() && dadk.cols() == a.cols());
        const auto dadk_ri = dadk.row(i);  // Eigen::Block
        double& drijk = drij[k];           // The element to calculate.
        for (int p = 0; p < a.cols(); ++p) {
          const double& dadk_rip = dadk_ri.coeffRef(p);
          const double& bjp = bj.coeffRef(p).value();
          drijk += dadk_rip * bjp;
        }
      }
    }
  }

  return result;
}

namespace internal {
// Base case: any type MatrixType without a better match is not a
// DifferentialMatrix.
template <typename MatrixType, typename = void>
struct is_diff_matrix_helper {
  static constexpr bool value = false;
};

// This partial specialization is preferred if MatrixType defines a boolean-
// convertible value IsDiffMatrix. We'll take that as convincing
// evidence that MatrixType is a DiffMatrix.
template <typename MatrixType>
struct is_diff_matrix_helper<MatrixType,
                             std::enable_if_t<MatrixType::IsDiffMatrix>> {
  static constexpr bool value = true;
};
}  // namespace internal

/** Reports at compile time whether the type supplied as a template argument
is a DiffMatrix. Use this if you have to specialize code differently for a
regular Eigen matrix (of double or symbolic::Expression) from a DiffMatrix that
is replacing what would have been `Matrix<AutoDiffXd>`.
@tparam MatrixType a type that might be a DiffMatrix
@relates DiffMatrix
*/
template <typename MatrixType>
inline static constexpr bool is_diff_matrix =
    internal::is_diff_matrix_helper<MatrixType>::value;

/** A type which is an ordinary Eigen Matrix if the scalar type T is double
or symbolic::Expression, but a DiffMatrix if T==AutoDiffXd. The template
parameters have the same meaning as Eigen's Matrix template parameters.
@see is_diff_matrix, MaybeDiffMatrixX
@relates DiffMatrix */
template <typename T, int rows, int cols, int options = 0, int max_rows = rows,
          int max_cols = cols>
using MaybeDiffMatrix = std::conditional_t<
    std::is_same_v<T, AutoDiffXd>,
    DiffMatrix<rows, cols, options, max_rows, max_cols>,
    Eigen::Matrix<T, rows, cols, options, max_rows, max_cols>>;

/** Convenience type like Drake's MatrixX that makes both row and column
dimensions dynamic. */
template <typename T>
using MaybeDiffMatrixX = MaybeDiffMatrix<T, Eigen::Dynamic, Eigen::Dynamic>;

/** Convenience type like Drake's MatrixX that makes both row and column
dimensions dynamic.
@relates DiffMatrix */
using DiffMatrixX = DiffMatrix<Eigen::Dynamic, Eigen::Dynamic>;

}  // namespace math
}  // namespace drake
