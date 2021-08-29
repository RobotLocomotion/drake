/// @file
/// Utilities that relate simultaneously to both autodiff matrices and
/// gradient matrices.

#pragma once

#include <algorithm>

#include <Eigen/Dense>
#include <fmt/format.h>

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
DRAKE_DEPRECATED("2021-12-01",
    "Apparently unused. File a Drake issue on GitHub if you need this method.")
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

/**
 * Differentiate a linear solver A * x = b. Where A is an Eigen matrix of
 * autodiffscalar, and b is an Eigen vector of autodiffscalar.
 * @tparam LinearSolverType The type of linear solver, for example
 * Eigen::LLT<Eigen::MatrixXd>. Notice that this is the linear solver for the
 * double version of A
 * @tparam DerivedA An Eigen Matrix of autodiffscalars.
 * @tparam DerivedB An Eigen Vector of autodiffscalars.
 * @param linear_solver A linear solver for the double version of A.
 * @param A The matrix A containing autodiff scalars.
 * @param b The vector b containing autodiff scalars.
 *
 * Here is an example code.
 * @code{cc}
 * Eigen::Matrix<AutoDiffXd, 2, 2> A_ad;
 * // Set the value and gradient in A_ad with arbitrary values;
 * A_ad(0, 0).value() = 1;
 * A_ad(0, 0).derivatives() = Eigen::Vector3d(1, 2, 3);
 * A_ad(0, 1).value() = 2;
 * A_ad(0, 1).derivatives() = Eigen::Vector3d(2, 3, 4);
 * A_ad(1, 0).value() = 3;
 * A_ad(1, 0).derivatives() = Eigen::Vector3d(3, 4, 5);
 * A_ad(1, 1).value() = 4;
 * A_ad(1, 1).derivatives() = Eigen::Vector3d(4, 5, 6);
 * Eigen::Matrix<AutoDiffXd, 2, 1> b_ad;
 * // Set the value and gradient in b_ad with arbitrary values.
 * b_ad(0).value() = 2;
 * b_ad(0).derivatives() = Eigen::Vector3d(1, 5, 8);
 * b_ad(1).value() = 3;
 * b_ad(1).derivatives() = Eigen::Vector3d(2, 1, 9);
 * // Get just the value of A_val
 * const Eigen::Matrix2d A_val = autoDiffToValueMatrix(A_ad);
 * // Construct the linear solver for A_val, namely just the double version of
 * // A.
 * const Eigen::SPartialPivLU<Eigen::Matrix2d> linear_solver(A_val);
 * // Solve the linear system A*x=b, together with the gradient.
 * // x_ad contains both the value of the solution A*x=b, together with its
 * // gradient.
 * const auto x_ad = DifferentiatieLinearSolving(linear_solver, A_ad, b_ad);
 * @endcode
 */
template <typename LinearSolverType, typename DerivedA, typename DerivedB>
typename std::enable_if<DerivedB::ColsAtCompileTime == 1,
                        Eigen::Matrix<typename DerivedA::Scalar,
                                      DerivedA::RowsAtCompileTime, 1>>::type
DifferentiateLinearSolving(const LinearSolverType& linear_solver,
                           const Eigen::MatrixBase<DerivedA>& A,
                           const Eigen::MatrixBase<DerivedB>& b) {
  // We differentiate A * x = b directly
  // A*∂x/∂z + ∂A/∂z * x = ∂b/∂z
  // So ∂x/∂z = A⁻¹(∂b/∂z - ∂A/∂z * x)
  // where I use z to denote the variable we take derivatives.

  // The size of the derivatives stored in A and b.
  int num_z_A = 0;
  int num_z_b = 0;
  for (int i = 0; i < A.rows(); ++i) {
    for (int j = 0; j < A.cols(); ++j) {
      if (A(i, j).derivatives().size() != 0) {
        if (num_z_A != 0 && A(i, j).derivatives().size() != 0 &&
            A(i, j).derivatives().size() != num_z_A) {
          throw std::runtime_error(fmt::format(
              "DifferentiateLinearSolving(): A({}, {}).derivatives() has size "
              "{}, while another entry has size {}",
              i, j, A(i, j).derivatives().size(), num_z_A));
        } else if (A(i, j).derivatives().size() != 0) {
          num_z_A = A(i, j).derivatives().size();
        }
      }
    }
    if (b(i).derivatives().size() != 0 && num_z_b != 0 &&
        b(i).derivatives().size() != num_z_b) {
      throw std::runtime_error(
          fmt::format("DifferentiateLinearSolving(): b({}).derivatives() has "
                      "size {}, while another entry has size {}",
                      i, b(i).derivatives().size(), num_z_b));
    } else if (b(i).derivatives().size() != 0) {
      num_z_b = b(i).derivatives().size();
    }
  }
  const auto b_val = autoDiffToValueMatrix(b);
  const Eigen::Matrix<double, DerivedA::RowsAtCompileTime, 1> x_val =
      linear_solver.solve(b_val);
  if (num_z_A == 0 && num_z_b == 0) {
    return x_val.template cast<typename DerivedA::Scalar>();
  } else if (num_z_A == 0) {
    // num_z_b != 0
    const auto b_grad = autoDiffToGradientMatrix(b);
    Eigen::Matrix<double, DerivedA::RowsAtCompileTime, Eigen::Dynamic> x_grad(
        A.rows(), num_z_b);
    for (int i = 0; i < num_z_b; ++i) {
      x_grad.col(i) = linear_solver.solve(b_grad.col(i));
    }
    return initializeAutoDiffGivenGradientMatrix(x_val, x_grad);
  } else {
    // num_z_A != 0
    if (num_z_A != 0 && num_z_b != 0 && num_z_A != num_z_b) {
      throw std::runtime_error(fmt::format(
          "DifferentiateLinearSolving(): A contains derivatives for {} "
          "variables, while b contains derivatives for {} variables",
          num_z_A, num_z_b));
    }
    Eigen::Matrix<double, DerivedA::RowsAtCompileTime, Eigen::Dynamic> x_grad(
        A.rows(), num_z_A);
    Eigen::Matrix<double, DerivedA::RowsAtCompileTime, Eigen::Dynamic> dAdzi(
        A.rows(), A.cols());
    dAdzi.setZero();
    Eigen::Matrix<double, DerivedA::RowsAtCompileTime, 1> dbdzi(A.rows());
    dbdzi.setZero();
    for (int i = 0; i < num_z_A; ++i) {
      // So ∂x/∂zᵢ = A⁻¹(∂b/∂zᵢ - ∂A/∂zᵢ * x)
      for (int j = 0; j < A.rows(); ++j) {
        for (int k = 0; k < A.cols(); ++k) {
          if (A(j, k).derivatives().rows() != 0) {
            dAdzi(j, k) = A(j, k).derivatives()(i);
          }
        }
        if (b(j).derivatives().rows() != 0) {
          dbdzi(j) = b(j).derivatives()(i);
        }
      }
      if (num_z_b > 0) {
        x_grad.col(i) = linear_solver.solve(dbdzi - dAdzi * x_val);
      } else {
        x_grad.col(i) = linear_solver.solve(-dAdzi * x_val);
      }
    }
    return initializeAutoDiffGivenGradientMatrix(x_val, x_grad);
  }
}

}  // namespace math
}  // namespace drake
