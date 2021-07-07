/// @file
/// Utilities that relate simultaneously to both autodiff matrices and
/// gradient matrices.

#pragma once

#include <algorithm>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
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

/** \brief Initializes an autodiff matrix given a matrix of values and gradient
 * matrix
 * \param[in] val value matrix
 * \param[in] gradient gradient matrix; the derivatives of val(j) are stored in
 * row j of the gradient matrix.
 * \param[out] autodiff_matrix matrix of AutoDiffScalars with the same size as
 * \p val
 */
template <typename Derived, typename DerivedGradient, typename DerivedAutoDiff>
void initializeAutoDiffGivenGradientMatrix(
    const Eigen::MatrixBase<Derived>& val,
    const Eigen::MatrixBase<DerivedGradient>& gradient,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
    Eigen::MatrixBase<DerivedAutoDiff>& auto_diff_matrix) {
  static_assert(static_cast<int>(Derived::SizeAtCompileTime) ==
                    static_cast<int>(DerivedGradient::RowsAtCompileTime),
                "gradient has wrong number of rows at compile time");
  DRAKE_ASSERT(val.size() == gradient.rows() &&
               "gradient has wrong number of rows at runtime");
  typedef AutoDiffMatrixType<Derived, DerivedGradient::ColsAtCompileTime>
      ExpectedAutoDiffType;
  static_assert(static_cast<int>(ExpectedAutoDiffType::RowsAtCompileTime) ==
                    static_cast<int>(DerivedAutoDiff::RowsAtCompileTime),
                "auto diff matrix has wrong number of rows at compile time");
  static_assert(static_cast<int>(ExpectedAutoDiffType::ColsAtCompileTime) ==
                    static_cast<int>(DerivedAutoDiff::ColsAtCompileTime),
                "auto diff matrix has wrong number of columns at compile time");
  static_assert(std::is_same_v<typename DerivedAutoDiff::Scalar,
                               typename ExpectedAutoDiffType::Scalar>,
                "wrong auto diff scalar type");

  typedef typename Eigen::MatrixBase<DerivedGradient>::Index Index;
  auto_diff_matrix.resize(val.rows(), val.cols());
  auto num_derivs = gradient.cols();
  for (Index row = 0; row < auto_diff_matrix.size(); row++) {
    auto_diff_matrix(row).value() = val(row);
    auto_diff_matrix(row).derivatives().resize(num_derivs, 1);
    auto_diff_matrix(row).derivatives() = gradient.row(row).transpose();
  }
}

/** \brief Creates and initializes an autodiff matrix given a matrix of values
 * and gradient matrix
 * \param[in] val value matrix
 * \param[in] gradient gradient matrix; the derivatives of val(j) are stored in
 * row j of the gradient matrix.
 * \return autodiff_matrix matrix of AutoDiffScalars with the same size as \p
 * val
 */
template <typename Derived, typename DerivedGradient>
AutoDiffMatrixType<Derived, DerivedGradient::ColsAtCompileTime>
initializeAutoDiffGivenGradientMatrix(
    const Eigen::MatrixBase<Derived>& val,
    const Eigen::MatrixBase<DerivedGradient>& gradient) {
  AutoDiffMatrixType<Derived, DerivedGradient::ColsAtCompileTime> ret(
      val.rows(), val.cols());
  initializeAutoDiffGivenGradientMatrix(val, gradient, ret);
  return ret;
}

template <typename DerivedGradient, typename DerivedAutoDiff>
void gradientMatrixToAutoDiff(
    const Eigen::MatrixBase<DerivedGradient>& gradient,
    // TODO(#2274) Fix NOLINTNEXTLINE(runtime/references).
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
 * AutoDiffScalars to AutoDiffScalar::Scalar type, but first checking that
 * the gradient matrix is empty or zero.  For a matrix of type, e.g.
 * `MatrixX<AutoDiffXd> A`, the comparable operation
 *   `B = A.cast<double>()`
 * should (and does) fail to compile.  Use `DiscardZeroGradient(A)` if you want
 * to force the cast (and the check).
 *
 * This method is overloaded to permit the user to call it for double types and
 * AutoDiffScalar types (to avoid the calling function having to handle the
 * two cases differently).
 *
 * @param precision is passed to Eigen's isZero(precision) to evaluate whether
 * the gradients are zero.
 * @throws std::exception if the gradients were not empty nor zero.
 *
 * @see DiscardGradient
 */
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
  const auto gradients = autoDiffToGradientMatrix(auto_diff_matrix);
  if (gradients.size() == 0 || gradients.isZero(precision)) {
    return autoDiffToValueMatrix(auto_diff_matrix);
  }
  throw std::runtime_error(
      "Casting AutoDiff to value but gradients are not zero.");
}

/// @see DiscardZeroGradient().
template <typename Derived>
typename std::enable_if_t<std::is_same_v<typename Derived::Scalar, double>,
                          const Eigen::MatrixBase<Derived>&>
DiscardZeroGradient(const Eigen::MatrixBase<Derived>& matrix,
                   double precision = 0.) {
  unused(precision);
  return matrix;
}

/// @see DiscardZeroGradient().
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

/// @see DiscardZeroGradient().
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
