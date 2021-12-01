/** @file
Utilities that relate simultaneously to both autodiff matrices and
gradient matrices. */

#pragma once

#include <algorithm>
#include <optional>

#include <Eigen/Dense>
#include <fmt/format.h>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/unused.h"
#include "drake/math/autodiff.h"
#include "drake/math/gradient.h"

namespace drake {
namespace math {


/** Extracts the `derivatives()` portion from a matrix of AutoDiffScalar
entries. (Each entry contains a value and derivatives.)

@param auto_diff_matrix An object whose Eigen type represents a matrix of
    AutoDiffScalar entries.
@param num_derivatives (Optional) The number of derivatives to return in case
    the input matrix has none, which we interpret as `num_derivatives` zeroes.
    If `num_derivatives` is supplied and the input matrix has derivatives, the
    sizes must match.
@retval gradient_matrix An Eigen::Matrix with number of rows equal to the
    total size (rows x cols) of the input matrix and number of columns equal
    to the number of derivatives. Each output row corresponds to one entry of
    the input matrix, in input row order.

@tparam Derived An Eigen type representing a matrix with AutoDiffScalar
    entries. The type will be inferred from the type of the `auto_diff_matrix`
    parameter at the call site.

@throws std::exception if the input matrix has elements with inconsistent,
    non-zero numbers of derivatives.
@throws std::exception if `num_derivatives` is specified but the input matrix
    has a different, non-zero number of derivatives.*/
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::SizeAtCompileTime,
              Eigen::Dynamic>
ExtractGradient(const Eigen::MatrixBase<Derived>& auto_diff_matrix,
                std::optional<int> num_derivatives = {}) {
  // Entries in an AutoDiff matrix must all have the same number of derivatives,
  // or 0-length derivatives in which case they are interpreted as all-zero.
  int num_derivatives_from_matrix = 0;
  for (int i = 0; i < auto_diff_matrix.size(); ++i) {
    const int entry_num_derivs =
        static_cast<int>(auto_diff_matrix(i).derivatives().size());
    if (entry_num_derivs == 0) continue;  // Always OK.
    if (num_derivatives_from_matrix != 0 &&
        entry_num_derivs != num_derivatives_from_matrix) {
      throw std::logic_error(fmt::format(
          "ExtractGradient(): Input matrix has elements with inconsistent,"
          " non-zero numbers of derivatives ({} and {}).",
          num_derivatives_from_matrix, entry_num_derivs));
    }
    num_derivatives_from_matrix = entry_num_derivs;
  }

  if (!num_derivatives.has_value()) {
    num_derivatives = num_derivatives_from_matrix;
  } else if (num_derivatives_from_matrix != 0 &&
             num_derivatives_from_matrix != *num_derivatives) {
    throw std::logic_error(fmt::format(
        "ExtractGradient(): Input matrix has {} derivatives, but"
        " num_derivatives was specified as {}. Either the input matrix should"
        " have zero derivatives, or the number should match num_derivatives.",
        num_derivatives_from_matrix, *num_derivatives));
  }

  Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::SizeAtCompileTime,
                Eigen::Dynamic>
      gradient(auto_diff_matrix.size(), *num_derivatives);
  for (int row = 0; row < auto_diff_matrix.rows(); ++row) {
    for (int col = 0; col < auto_diff_matrix.cols(); ++col) {
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

template <typename Derived>
struct DRAKE_DEPRECATED("2022-02-01", "Used only in deprecated functions.")
    AutoDiffToGradientMatrix {
  typedef typename Gradient<
      Eigen::Matrix<typename Derived::Scalar::Scalar,
                    Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>,
      Eigen::Dynamic>::type type;
};

// This deprecated function uses the above deprecated struct.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
template <typename Derived>
DRAKE_DEPRECATED("2022-02-01", "Use ExtractGradient().")
typename AutoDiffToGradientMatrix<Derived>::type autoDiffToGradientMatrix(
    const Eigen::MatrixBase<Derived>& autodiff_matrix,
    int num_derivatives = Eigen::Dynamic) {
  return ExtractGradient(autodiff_matrix,
                         num_derivatives == Eigen::Dynamic
                             ? std::nullopt
                             : std::optional<int>(num_derivatives));
}
#pragma GCC diagnostic pop

/** Initializes an AutoDiff matrix given a matrix of values and a gradient
matrix.

@param[in] value The value matrix. Will be accessed with a single index.
@param[in] gradient The gradient matrix. The number of rows must match the
    total size (nrow x ncol) of the value matrix. Derivatives of value(j) should
    be stored in row j of the gradient matrix.
@param[out] auto_diff_matrix The matrix of AutoDiffScalars. Will be resized as
    needed to have the same dimensions as the value matrix.
@exclude_from_pydrake_mkdoc{Not bound in pydrake.} */
template <typename DerivedValue, typename DerivedGradient,
          typename DerivedAutoDiff>
void InitializeAutoDiff(
    const Eigen::MatrixBase<DerivedValue>& value,
    const Eigen::MatrixBase<DerivedGradient>& gradient,
    Eigen::MatrixBase<DerivedAutoDiff>* auto_diff_matrix) {

  // Any fixed-size dimensions of the parameters must be consistent in
  // corresponding dimensions. Any dynamic-size dimensions must be dynamic
  // in any corresponding dimensions.
  static_assert(static_cast<int>(DerivedValue::SizeAtCompileTime) ==
                    static_cast<int>(DerivedGradient::RowsAtCompileTime),
                "gradient has wrong number of rows at compile time");
  using ExpectedAutoDiffType =
      AutoDiffMatrixType<DerivedValue, DerivedGradient::ColsAtCompileTime>;
  static_assert(static_cast<int>(ExpectedAutoDiffType::RowsAtCompileTime) ==
                    static_cast<int>(DerivedAutoDiff::RowsAtCompileTime),
                "auto diff matrix has wrong number of rows at compile time");
  static_assert(static_cast<int>(ExpectedAutoDiffType::ColsAtCompileTime) ==
                    static_cast<int>(DerivedAutoDiff::ColsAtCompileTime),
                "auto diff matrix has wrong number of columns at compile time");

  // Verify that the Scalar types of the Value matrix and AutoDiff result
  // are the same.
  static_assert(std::is_same_v<typename DerivedAutoDiff::Scalar,
                               typename ExpectedAutoDiffType::Scalar>,
                "wrong auto diff scalar type");

  DRAKE_DEMAND(auto_diff_matrix != nullptr);
  DRAKE_DEMAND(value.size() == gradient.rows() &&
               "gradient has wrong number of rows at runtime");

  using Index = typename Eigen::MatrixBase<DerivedGradient>::Index;

  // Can't resize() a MatrixBase -- downcast to the actual type.
  DerivedAutoDiff& auto_diff = *static_cast<DerivedAutoDiff*>(auto_diff_matrix);
  auto_diff.resize(value.rows(), value.cols());

  auto num_derivs = gradient.cols();
  for (Index row = 0; row < auto_diff.size(); ++row) {
    auto_diff(row).value() = value(row);
    auto_diff(row).derivatives().resize(num_derivs, 1);
    auto_diff(row).derivatives() = gradient.row(row).transpose();
  }
}

template <typename DerivedValue, typename DerivedGradient,
          typename DerivedAutoDiff>
DRAKE_DEPRECATED("2022-02-01", "Use InitializeAutoDiff().")
void initializeAutoDiffGivenGradientMatrix(
    const Eigen::MatrixBase<DerivedValue>& value,
    const Eigen::MatrixBase<DerivedGradient>& gradient,
    // NOLINTNEXTLINE(runtime/references).
    Eigen::MatrixBase<DerivedAutoDiff>& auto_diff_matrix) {
  InitializeAutoDiff(value, gradient, &auto_diff_matrix);
}

/** Returns an AutoDiff matrix given a matrix of values and a gradient
matrix.

@param[in] value The value matrix. Will be accessed with a single index.
@param[in] gradient The gradient matrix. The number of rows must match the
    total size (nrow x ncol) of the value matrix. Derivatives of value(j) should
    be stored in row j of the gradient matrix.
@retval auto_diff_matrix The matrix of AutoDiffScalars. Will have the same
    dimensions as the value matrix.
@pydrake_mkdoc_identifier{value_and_gradient} */
template <typename DerivedValue, typename DerivedGradient>
AutoDiffMatrixType<DerivedValue, DerivedGradient::ColsAtCompileTime>
InitializeAutoDiff(
    const Eigen::MatrixBase<DerivedValue>& value,
    const Eigen::MatrixBase<DerivedGradient>& gradient) {
  AutoDiffMatrixType<DerivedValue, DerivedGradient::ColsAtCompileTime>
      auto_diff_matrix(value.rows(), value.cols());
  InitializeAutoDiff(value, gradient, &auto_diff_matrix);
  return auto_diff_matrix;
}

template <typename DerivedValue, typename DerivedGradient>
DRAKE_DEPRECATED("2022-02-01", "Use InitializeAutoDiff() instead")
AutoDiffMatrixType<DerivedValue, DerivedGradient::ColsAtCompileTime>
initializeAutoDiffGivenGradientMatrix(
    const Eigen::MatrixBase<DerivedValue>& val,
    const Eigen::MatrixBase<DerivedGradient>& gradient) {
  return InitializeAutoDiff(val, gradient);
}

/** `B = DiscardZeroGradient(A, precision)` enables casting from a matrix of
AutoDiffScalars to AutoDiffScalar::Scalar type, but first checking that
the gradient matrix is empty or zero.  For a matrix of type, e.g.
`MatrixX<AutoDiffXd> A`, the comparable operation
  `B = A.cast<double>()`
should (and does) fail to compile.  Use `DiscardZeroGradient(A)` if you want
to force the cast (and the check).

This method is overloaded to permit the user to call it for double types and
AutoDiffScalar types (to avoid the calling function having to handle the
two cases differently).

See ExtractValue() for a note on similar Drake functions.

@param precision is passed to Eigen's isZero(precision) to evaluate whether
the gradients are zero.
@throws std::exception if the gradients were not empty nor zero.
@see DiscardGradient() */
template <typename Derived>
typename std::enable_if_t<
    !std::is_same_v<typename Derived::Scalar, double>,
    Eigen::Matrix<typename Derived::Scalar::Scalar, Derived::RowsAtCompileTime,
                  Derived::ColsAtCompileTime, 0, Derived::MaxRowsAtCompileTime,
                  Derived::MaxColsAtCompileTime>>
DiscardZeroGradient(
    const Eigen::MatrixBase<Derived>& auto_diff_matrix,
    const typename Eigen::NumTraits<
        typename Derived::Scalar::Scalar>::Real& precision =
        Eigen::NumTraits<typename Derived::Scalar::Scalar>::dummy_precision()) {
  const auto gradients = ExtractGradient(auto_diff_matrix);
  if (gradients.size() == 0 || gradients.isZero(precision)) {
    return ExtractValue(auto_diff_matrix);
  }
  throw std::runtime_error(
      "Casting AutoDiff to value but gradients are not zero.");
}

/** @see DiscardZeroGradient(). */
template <typename Derived>
typename std::enable_if_t<std::is_same_v<typename Derived::Scalar, double>,
                          const Eigen::MatrixBase<Derived>&>
DiscardZeroGradient(const Eigen::MatrixBase<Derived>& matrix,
                   double precision = 0.) {
  unused(precision);
  return matrix;
}

/**
 * Given a matrix of AutoDiffScalars, returns the size of the
 * derivatives.
 * @throw runtime_error if some entry has different (non-zero) number of
 * derivatives as the others.
 */
template <typename Derived>
typename std::enable_if<!std::is_same_v<typename Derived::Scalar, double>,
                        int>::type
GetDerivativeSize(const Eigen::MatrixBase<Derived>& A) {
  int num_derivs = 0;
  for (int i = 0; i < A.rows(); ++i) {
    for (int j = 0; j < A.cols(); ++j) {
      if (A(i, j).derivatives().size() != 0) {
        if (num_derivs != 0 && A(i, j).derivatives().size() != num_derivs) {
          throw std::runtime_error(fmt::format(
              "GetDerivativeSize(): A({}, {}).derivatives() has size "
              "{}, while another entry has size {}",
              i, j, A(i, j).derivatives().size(), num_derivs));
        }
        num_derivs = A(i, j).derivatives().size();
      }
    }
  }
  return num_derivs;
}

}  // namespace math
}  // namespace drake
