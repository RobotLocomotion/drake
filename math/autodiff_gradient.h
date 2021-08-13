/** @file
Utilities that relate simultaneously to both autodiff matrices and
gradient matrices. */

#pragma once

#include <algorithm>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/unused.h"
#include "drake/math/autodiff.h"
#include "drake/math/gradient.h"

namespace drake {
namespace math {

template <typename Derived>
struct AutoDiffToGradientMatrix {
  typedef typename Gradient<
      Eigen::Matrix<typename Derived::Scalar::Scalar,
                    Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>,
      Eigen::Dynamic>::type type;
};

template <typename Derived>
typename AutoDiffToGradientMatrix<Derived>::type
ExtractGradientMatrixFromAutoDiff(
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

template <typename Derived>
// TODO(sherm1) DRAKE_DEPRECATED("2021-12-01",
//     "Use ExtractGradientMatrixFromAutoDiff().")
typename AutoDiffToGradientMatrix<Derived>::type autoDiffToGradientMatrix(
    const Eigen::MatrixBase<Derived>& auto_diff_matrix,
    int num_variables = Eigen::Dynamic) {
  return ExtractGradientMatrixFromAutoDiff(auto_diff_matrix, num_variables);
}

/** Initializes an AutoDiff matrix given a matrix of values and a gradient
matrix.

@param[in] value The value matrix. Will be accessed with a single index.
@param[in] gradient The gradient matrix. The number of rows must match the
    total size (nrow x ncol) of the value matrix. Derivatives of value(j) should
    be stored in row j of the gradient matrix.
@param[out] autodiff_matrix The matrix of AutoDiffScalars. Will be resized as
    needed to have the same dimensions as the value matrix. */
template <typename DerivedValue, typename DerivedGradient,
          typename DerivedAutoDiff>
void InitializeAutoDiffFromValueAndGradientMatrix(
    const Eigen::MatrixBase<DerivedValue>& value,
    const Eigen::MatrixBase<DerivedGradient>& gradient,
    Eigen::MatrixBase<DerivedAutoDiff>* auto_diff_matrix) {
  // Verify either that we have conformant fixed-sized matrices, or that they
  // are dynamically sized.
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
  auto_diff_matrix->resize(value.rows(), value.cols());
  auto num_derivs = gradient.cols();
  for (Index row = 0; row < auto_diff_matrix->size(); row++) {
    (*auto_diff_matrix)(row).value() = value(row);
    (*auto_diff_matrix)(row).derivatives().resize(num_derivs, 1);
    (*auto_diff_matrix)(row).derivatives() = gradient.row(row).transpose();
  }
}

template <typename DerivedValue, typename DerivedGradient,
          typename DerivedAutoDiff>
// TODO(sherm1) DRAKE_DEPRECATED("2021-12-01",
//     "Use InitializeAutoDiffFromValueAndGradientMatrix().")
void initializeAutoDiffGivenGradientMatrix(
    const Eigen::MatrixBase<DerivedValue>& value,
    const Eigen::MatrixBase<DerivedGradient>& gradient,
    // NOLINTNEXTLINE(runtime/references).
    Eigen::MatrixBase<DerivedAutoDiff>& auto_diff_matrix) {
  InitializeAutoDiffFromValueAndGradientMatrix(value, gradient,
                                               &auto_diff_matrix);
}

/** Returns an AutoDiff matrix given a matrix of values and a gradient
matrix.

@param[in] value The value matrix. Will be accessed with a single index.
@param[in] gradient The gradient matrix. The number of rows must match the
    total size (nrow x ncol) of the value matrix. Derivatives of value(j) should
    be stored in row j of the gradient matrix.
@retval autodiff_matrix The matrix of AutoDiffScalars. Will be resized as
    needed to have the same dimensions as the value matrix. */
template <typename DerivedValue, typename DerivedGradient>
AutoDiffMatrixType<DerivedValue, DerivedGradient::ColsAtCompileTime>
InitializeAutoDiffFromValueAndGradientMatrix(
    const Eigen::MatrixBase<DerivedValue>& value,
    const Eigen::MatrixBase<DerivedGradient>& gradient) {
  AutoDiffMatrixType<DerivedValue, DerivedGradient::ColsAtCompileTime>
      auto_diff_matrix(value.rows(), value.cols());
  InitializeAutoDiffFromValueAndGradientMatrix(value, gradient,
                                               &auto_diff_matrix);
  return auto_diff_matrix;
}

template <typename DerivedValue, typename DerivedGradient>
// TODO(sherm1) DRAKE_DEPRECATED("2021-12-01",
//     "Use InitializeAutoDiffFromValueAndGradientMatrix() instead")
AutoDiffMatrixType<DerivedValue, DerivedGradient::ColsAtCompileTime>
initializeAutoDiffGivenGradientMatrix(
    const Eigen::MatrixBase<DerivedValue>& value,
    const Eigen::MatrixBase<DerivedGradient>& gradient) {
  return InitializeAutoDiffFromValueAndGradientMatrix(value, gradient);
}

template <typename DerivedGradient, typename DerivedAutoDiff>
DRAKE_DEPRECATED("2021-12-01",
                 "Use InitializeAutoDiffFromValueAndGradientMatrix() instead")
void gradientMatrixToAutoDiff(
    const Eigen::MatrixBase<DerivedGradient>& gradient,
    // NOLINTNEXTLINE(runtime/references).
    Eigen::MatrixBase<DerivedAutoDiff>& auto_diff_matrix) {
  typedef typename Eigen::MatrixBase<DerivedGradient>::Index Index;
  auto nx = gradient.cols();
  for (Index row = 0; row < auto_diff_matrix.rows(); row++) {
    for (Index col = 0; col < auto_diff_matrix.cols(); col++) {
      auto_diff_matrix(row, col).derivatives().resize(nx, 1);
      auto_diff_matrix(row, col).derivatives() =
          gradient.row(row + col * auto_diff_matrix.rows()).transpose();
    }
  }
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
  const auto gradients = ExtractGradientMatrixFromAutoDiff(auto_diff_matrix);
  if (gradients.size() == 0 || gradients.isZero(precision)) {
    return ExtractValueMatrixFromAutoDiff(auto_diff_matrix);
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

/** @see DiscardZeroGradient(). */
template <typename _Scalar, int _Dim, int _Mode, int _Options>
typename std::enable_if_t<
    !std::is_same_v<_Scalar, double>,
    Eigen::Transform<typename _Scalar::Scalar, _Dim, _Mode, _Options>>
DiscardZeroGradient(
    const Eigen::Transform<_Scalar, _Dim, _Mode, _Options>& auto_diff_transform,
    const typename Eigen::NumTraits<typename _Scalar::Scalar>::Real& precision =
        Eigen::NumTraits<typename _Scalar::Scalar>::dummy_precision()) {
  return Eigen::Transform<typename _Scalar::Scalar, _Dim, _Mode, _Options>(
      DiscardZeroGradient(auto_diff_transform.matrix(), precision));
}

/** @see DiscardZeroGradient(). */
template <typename _Scalar, int _Dim, int _Mode, int _Options>
typename std::enable_if_t<
    std::is_same_v<_Scalar, double>,
    const Eigen::Transform<_Scalar, _Dim, _Mode, _Options>&>
DiscardZeroGradient(
    const Eigen::Transform<_Scalar, _Dim, _Mode, _Options>& transform,
    double precision = 0.) {
  unused(precision);
  return transform;
}

}  // namespace math
}  // namespace drake
