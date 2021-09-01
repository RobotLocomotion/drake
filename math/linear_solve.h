#pragma once

#include "math/autodiff.h"
#include "math/autodiff_gradient.h"

namespace drake {
namespace math {
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
