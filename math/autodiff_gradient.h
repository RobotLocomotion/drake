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
        if (num_derivs != 0 && A(i, j).derivatives().size() != 0 &&
            A(i, j).derivatives().size() != num_derivs) {
          throw std::runtime_error(fmt::format(
              "GetDerivativeSize(): A({}, {}).derivatives() has size "
              "{}, while another entry has size {}",
              i, j, A(i, j).derivatives().size(), num_derivs));
        } else if (A(i, j).derivatives().size() != 0) {
          num_derivs = A(i, j).derivatives().size();
        }
      }
    }
  }
  return num_derivs;
}

/**
 * @anchor linear_solve
 * @name solve linear system of equations
 * Solve linear system of equations A * x = b. Where A is an Eigen matrix of
 * AutoDiffScalar, and b is an Eigen vector of AutoDiffScalar.
 * @tparam LinearSolverType The type of linear solver, for example
 * Eigen::LLT. Notice that this is just specifies the solver type (such as
 * Eigen::LLT), not the matrix type (like Eigen::LLT<Eigen::Matrix2d>).
 * @tparam DerivedA An Eigen Matrix.
 * @tparam DerivedB An Eigen Vector.
 * @param A The matrix A.
 * @param b The vector b.
 *
 * Here is an example code.
 * @code{cc}
 * Eigen::Matrix<AutoDiffd<3>, 2, 2> A_ad;
 * // Set the value and gradient in A_ad with arbitrary values;
 * Eigen::Matrix2d A_val;
 * A_val << 1, 2, 3, 4;
 * // Gradient of A.col(0).
 * Eigen::Matrix<double, 2, 3> A0_gradient;
 * A0_gradient << 1, 2, 3, 4, 5, 6;
 * A_ad.col(0) = initializeAutoDiffGivenGradientMatrix(
 *     A_val.col(0), A0_gradient);
 * // Gradient of A.col(1)
 * Eigen::Matrix<double, 2, 3> A1_gradient;
 * A1_gradient << 7, 8, 9, 10, 11, 12;
 * A_ad.col(1) = initializeAutoDiffGivenGradientMatrix(
 *     A_val.col(1), A1_gradient);
 * // Set the value and gradient of b to arbitrary value.
 * const Eigen::Vector2d b_val(2, 3);
 * Eigen::Matrix<double, 2, 3> b_gradient;
 * b_gradient << 1, 3, 5, 7, 9, 11;
 * const auto b_ad = initializeAutoDiffGivenGradientMatrix(b_val, b_gradient);
 * // Solve the linear system A*x=b, together with the gradient.
 * // x_ad contains both the value of the solution A*x=b, together with its
 * // gradient.
 * const auto x_ad = LinearSolve<Eigen::PartialPivLU>(A_ad, b_ad);
 * @endcode
 */

//@{
/**
 * Template specialization for both A and b being double-valued matrices.
 * See @ref linear_solve for more details.
 */
template <template <typename> typename LinearSolverType, typename DerivedA,
          typename DerivedB>
typename std::enable_if<
    DerivedB::ColsAtCompileTime == 1 &&
        std::is_same_v<typename DerivedA::Scalar, double> &&
        std::is_same_v<typename DerivedB::Scalar, double>,
    Eigen::Matrix<double, DerivedA::RowsAtCompileTime, 1>>::type
LinearSolve(const Eigen::MatrixBase<DerivedA>& A,
            const Eigen::MatrixBase<DerivedB>& b) {
  const LinearSolverType<Eigen::Matrix<double, DerivedA::RowsAtCompileTime,
                                       DerivedA::ColsAtCompileTime>>
      linear_solver(A);
  return linear_solver.solve(b);
}

/**
 * Template specialization for A being double-valued matrix, and b being
 * AutoDiffScalar-valued vector. See @ref linear_solve for more details.
 */
template <template <typename> typename LinearSolverType, typename DerivedA,
          typename DerivedB>
typename std::enable_if<DerivedB::ColsAtCompileTime == 1 &&
                            std::is_same_v<typename DerivedA::Scalar, double> &&
                            !std::is_same_v<typename DerivedB::Scalar, double>,
                        Eigen::Matrix<typename DerivedB::Scalar,
                                      DerivedA::RowsAtCompileTime, 1>>::type
LinearSolve(const Eigen::MatrixBase<DerivedA>& A,
            const Eigen::MatrixBase<DerivedB>& b) {
  const int num_z_b = GetDerivativeSize(b);
  if (num_z_b == 0) {
    return LinearSolve<LinearSolverType>(A, autoDiffToValueMatrix(b))
        .template cast<typename DerivedB::Scalar>();
  }
  const LinearSolverType<Eigen::Matrix<double, DerivedA::RowsAtCompileTime,
                                       DerivedA::ColsAtCompileTime>>
      linear_solver(A);
  const Eigen::Matrix<double, DerivedA::RowsAtCompileTime, 1> x_val =
      linear_solver.solve(autoDiffToValueMatrix(b));
  auto grad = autoDiffToGradientMatrix(b);
  // At the beginning of the loop, grad stores the gradient of b. At the end of
  // the loop, grad stores the gradient of x. I don't create a separate matrix
  // for the gradient of x, as that might incur memory heap allocation.
  for (int i = 0; i < num_z_b; ++i) {
    grad.col(i) = linear_solver.solve(grad.col(i));
  }
  return initializeAutoDiffGivenGradientMatrix(x_val, grad);
}

/**
 * Template specialization when A is a matrix of AutoDiffScalar.
 * See @ref linear_solve for more details.
 */
template <template <typename> typename LinearSolverType, typename DerivedA,
          typename DerivedB>
typename std::enable_if<DerivedB::ColsAtCompileTime == 1 &&
                            !std::is_same_v<typename DerivedA::Scalar, double>,
                        Eigen::Matrix<typename DerivedA::Scalar,
                                      DerivedA::RowsAtCompileTime, 1>>::type
LinearSolve(const Eigen::MatrixBase<DerivedA>& A,
            const Eigen::MatrixBase<DerivedB>& b) {
  // We differentiate A * x = b directly
  // A*∂x/∂zᵢ + ∂A/∂zᵢ * x = ∂b/∂zᵢ
  // So ∂x/∂zᵢ = A⁻¹(∂b/∂zᵢ - ∂A/∂zᵢ * x)
  // where I use z to denote the variable we take derivatives.

  // The size of the derivatives stored in A and b.
  const int num_z_A = GetDerivativeSize(A);
  int num_z_b = 0;
  if constexpr (!std::is_same_v<typename DerivedB::Scalar, double>) {
    num_z_b = GetDerivativeSize(b);
  }
  if (num_z_A == 0 && num_z_b == 0) {
    if constexpr (std::is_same_v<typename DerivedB::Scalar, double>) {
      return LinearSolve<LinearSolverType>(autoDiffToValueMatrix(A), b)
          .template cast<typename DerivedA::Scalar>();
    } else {
      return LinearSolve<LinearSolverType>(autoDiffToValueMatrix(A),
                                           autoDiffToValueMatrix(b))
          .template cast<typename DerivedA::Scalar>();
    }
  } else if (num_z_A == 0 && num_z_b != 0) {
    return LinearSolve<LinearSolverType>(autoDiffToValueMatrix(A), b);
  }
  // First compute the value of x.
  const LinearSolverType<Eigen::Matrix<double, DerivedA::RowsAtCompileTime,
                                       DerivedA::ColsAtCompileTime>>
      linear_solver(autoDiffToValueMatrix(A));
  Eigen::Matrix<double, DerivedA::RowsAtCompileTime, 1> x_val;
  if constexpr (std::is_same_v<typename DerivedB::Scalar, double>) {
    x_val = linear_solver.solve(b);
  } else {
    const auto b_val = autoDiffToValueMatrix(b);
    x_val = linear_solver.solve(b_val);
  }

  // num_z_A != 0
  if (num_z_A != 0 && num_z_b != 0 && num_z_A != num_z_b) {
    throw std::runtime_error(
        fmt::format("LinearSolve(): A contains derivatives for {} "
                    "variables, while b contains derivatives for {} variables",
                    num_z_A, num_z_b));
  }
  Eigen::Matrix<double, DerivedA::RowsAtCompileTime, Eigen::Dynamic> x_grad(
      A.rows(), num_z_A);
  Eigen::Matrix<double, DerivedA::RowsAtCompileTime, Eigen::Dynamic> dAdzi(
      A.rows(), A.cols());
  Eigen::Matrix<double, DerivedA::RowsAtCompileTime, 1> dbdzi(A.rows());
  for (int i = 0; i < num_z_A; ++i) {
    dAdzi.setZero();
    dbdzi.setZero();
    // So ∂x/∂zᵢ = A⁻¹(∂b/∂zᵢ - ∂A/∂zᵢ * x)
    for (int j = 0; j < A.rows(); ++j) {
      for (int k = 0; k < A.cols(); ++k) {
        if (A(j, k).derivatives().rows() != 0) {
          dAdzi(j, k) = A(j, k).derivatives()(i);
        }
      }
      if constexpr (std::is_same_v<typename DerivedB::Scalar, double>) {
        dbdzi(j) = 0.;
      } else {
        if (b(j).derivatives().rows() != 0) {
          dbdzi(j) = b(j).derivatives()(i);
        }
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
//@}

}  // namespace math
}  // namespace drake
