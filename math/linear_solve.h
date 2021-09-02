#pragma once

#include "common/symbolic.h"
#include "math/autodiff.h"
#include "math/autodiff_gradient.h"

namespace drake {
namespace math {
namespace internal {
template <typename T>
struct is_double_or_symbolic : std::false_type {};

template <>
struct is_double_or_symbolic<double> : std::true_type {};

template <>
struct is_double_or_symbolic<symbolic::Expression> : std::true_type {};

template <>
struct is_double_or_symbolic<symbolic::Variable> : std::true_type {};
}  // namespace internal

/**
 * @anchor linear_solve_given_solver
 * @name solve linear system of equations with a given solver.
 * Solve linear system of equations A * x = b. Where A is an Eigen matrix of
 * double/AutoDiffScalar, and b is an Eigen matrix of double/AutoDiffScalar.
 * Notice that we don't support symbolic variable/expression in A or b.
 * This 3-argument version allows the user to re-use @p linear_solver when @p b
 * changes or the gradient of @p A changes.
 * When either A or b contains AutoDiffScalar, we use implicit function theorem
 * to find the gradient in x as ∂x/∂zᵢ = A⁻¹(∂b/∂zᵢ - ∂A/∂zᵢ * x) where z is the
 * variable we take gradient with.
 *
 * @note When both A and b are Eigen matrix of double, this function is almost
 * as fast as calling linear_solver.solve(b) directly. When either A or b
 * contains AutoDiffScalar, this function is a lot faster than first
 * instantiating the linear solver of AutoDiffScalar, and then solving the
 * equation with this autodiffable linear solver.
 * @tparam LinearSolver The type of linear solver, for example
 * Eigen::LLT<Eigen::Matrix2d>
 * @tparam DerivedA An Eigen Matrix.
 * @tparam DerivedB An Eigen Vector.
 * @param linear_solver The linear solver constructed with the double-version of
 * A.
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
 * // Solve the linear system A_val * x_val = b_val.
 * Eigen::PartialPivLU<Eigen::Matrix2d> linear_solver(A_val);
 * const auto x_val = LinearSolve(linear_solver, A_val, b_val);
 * // Solve the linear system A*x=b, together with the gradient.
 * // x_ad contains both the value of the solution A*x=b, together with its
 * // gradient.
 * const auto x_ad = LinearSolve(linear_solver, A_ad, b_ad);
 * @endcode
 */

//@{
/**
 * Specialized when A and b are both double-valued matrices. See @ref
 * linear_solve_given_solver for more details.
 * Note that @p A is unused, as we already compute its factorization in @p
 * linear_solver. But we keep it here for consistency with the overloaded
 * function, where A is a matrix of AutoDiffScalar.
 */
template <typename LinearSolver, typename DerivedA, typename DerivedB>
typename std::enable_if<
    DerivedB::ColsAtCompileTime == 1 &&
        std::is_same_v<typename DerivedA::Scalar, double> &&
        std::is_same_v<typename DerivedB::Scalar, double>,
    Eigen::Matrix<double, DerivedA::RowsAtCompileTime, 1>>::type
LinearSolve(const LinearSolver& linear_solver,
            const Eigen::MatrixBase<DerivedA>& A,
            const Eigen::MatrixBase<DerivedB>& b) {
  unused(A);
  return linear_solver.solve(b);
}

/**
 * Specialized when A is a double-valued matrix and b is an
 * AutoDiffScalar-valued matrix .See @ref linear_solve_given_solver for more
 * details. Note that @p A is unused, as we already compute its factorization in
 * @p linear_solver. But we keep it here for consistency with the overloaded
 * function, where A is a matrix of AutoDiffScalar.
 */
template <typename LinearSolver, typename DerivedA, typename DerivedB>
typename std::enable_if<
    DerivedB::ColsAtCompileTime == 1 &&
        std::is_same_v<typename DerivedA::Scalar, double> &&
        !internal::is_double_or_symbolic<typename DerivedB::Scalar>::value,
    Eigen::Matrix<typename DerivedB::Scalar, DerivedA::RowsAtCompileTime,
                  1>>::type
LinearSolve(const LinearSolver& linear_solver,
            const Eigen::MatrixBase<DerivedA>& A,
            const Eigen::MatrixBase<DerivedB>& b) {
  unused(A);
  const int num_z_b = GetDerivativeSize(b);
  if (num_z_b == 0) {
    return linear_solver.solve(autoDiffToValueMatrix(b))
        .template cast<typename DerivedB::Scalar>();
  }
  const Eigen::Matrix<double, DerivedA::RowsAtCompileTime, 1> x_val =
      linear_solver.solve(autoDiffToValueMatrix(b));
  const auto b_grad = autoDiffToGradientMatrix(b);
  // ∂x/∂zᵢ = A⁻¹*∂b/∂zᵢ
  // To avoid aliasing issue, I create an x_grad matrix. If we are 100% sure
  // that calling solve won't cause the aliasing issue, then we can use a single
  // grad matrix, and call
  // auto grad = autoDiffToGradientMatrix(b);
  // grad = linear_solver.solve(grad);
  // return initializeAutoDiffGivenGradientMatrix(x_val, grad);
  // which avoids creating x_grad with potential heap memory allocation.
  const auto x_grad = linear_solver.solve(b_grad);
  return initializeAutoDiffGivenGradientMatrix(x_val, x_grad);
}

/**
 * Specialized when both A is an AutoDiffScalar-valued matrix, and b can contain
 * either AutoDiffScalar or double.See @ref linear_solve_given_solver for more
 * details.
 */
template <typename LinearSolver, typename DerivedA, typename DerivedB>
typename std::enable_if<
    DerivedB::ColsAtCompileTime == 1 &&
        !internal::is_double_or_symbolic<typename DerivedA::Scalar>::value,
    Eigen::Matrix<typename DerivedA::Scalar, DerivedA::RowsAtCompileTime,
                  1>>::type
LinearSolve(const LinearSolver& linear_solver,
            const Eigen::MatrixBase<DerivedA>& A,
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
      return LinearSolve(linear_solver, autoDiffToValueMatrix(A), b)
          .template cast<typename DerivedA::Scalar>();
    } else {
      return LinearSolve(linear_solver, autoDiffToValueMatrix(A),
                         autoDiffToValueMatrix(b))
          .template cast<typename DerivedA::Scalar>();
    }
  } else if (num_z_A == 0 && num_z_b != 0) {
    return LinearSolve(linear_solver, autoDiffToValueMatrix(A), b);
  }
  // First compute the value of x.
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
    // First we need to store the matrix ∂A/∂zᵢ
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
    x_grad.col(i) = linear_solver.solve(dbdzi - dAdzi * x_val);
  }
  return initializeAutoDiffGivenGradientMatrix(x_val, x_grad);
}

//@}

/**
 * @anchor linear_solve
 * @name solve linear system of equations
 * Solve linear system of equations A * x = b. Where A is an Eigen matrix of
 * double/AutoDiffScalar, and b is an Eigen matrix of double/AutoDiffScalar.
 * Notice that we don't support symbolic variable/expression in A or b.
 * When either A or b contains AutoDiffScalar, we use implicit function theorem
 * to find the gradient in x as ∂x/∂zᵢ = A⁻¹(∂b/∂zᵢ - ∂A/∂zᵢ * x) where z is the
 * variable we take gradient with.
 *
 * @note When both A and b are Eigen matrix of double, this function is almost
 * as fast as calling linear_solver.solve(b) directly; when either A or b
 * contains AutoDiffScalar, this function is a lot faster than first
 * instantiating the linear solver of AutoDiffScalar, and then solving the
 * equation with this autodiffable linear solver.
 * @tparam LinearSolverType The type of linear solver, for example
 * Eigen::LLT. Notice that this is just specifies the solver type (such as
 * Eigen::LLT), not the matrix type (like Eigen::LLT<Eigen::Matrix2d>).
 * All Eigen solvers we care about are templated on the matrix type. Some are
 * further templated on configuration ints. The int... will acount for zero or
 * more of these ints, providing a common interface for both types of solvers.
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
 * // Solve the linear system A*x=b without the gradient.
 * const auto x_val = LinearSolve<Eigen::PartialPivLU>(A_val, b_val);
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
template <template <typename, int...> typename LinearSolverType,
          typename DerivedA, typename DerivedB>
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
  return LinearSolve(linear_solver, A, b);
}

/**
 * Template specialization for A being double-valued matrix, and b being
 * AutoDiffScalar-valued matrix. See @ref linear_solve for more details.
 */
template <template <typename, int...> typename LinearSolverType,
          typename DerivedA, typename DerivedB>
typename std::enable_if<
    DerivedB::ColsAtCompileTime == 1 &&
        std::is_same_v<typename DerivedA::Scalar, double> &&
        !internal::is_double_or_symbolic<typename DerivedB::Scalar>::value,
    Eigen::Matrix<typename DerivedB::Scalar, DerivedA::RowsAtCompileTime,
                  1>>::type
LinearSolve(const Eigen::MatrixBase<DerivedA>& A,
            const Eigen::MatrixBase<DerivedB>& b) {
  const LinearSolverType<Eigen::Matrix<double, DerivedA::RowsAtCompileTime,
                                       DerivedA::ColsAtCompileTime>>
      linear_solver(A);
  return LinearSolve(linear_solver, A, b);
}

/**
 * Template specialization when A is a matrix of AutoDiffScalar.
 * See @ref linear_solve for more details.
 */
template <template <typename, int...> typename LinearSolverType,
          typename DerivedA, typename DerivedB>
typename std::enable_if<
    DerivedB::ColsAtCompileTime == 1 &&
        !internal::is_double_or_symbolic<typename DerivedA::Scalar>::value,
    Eigen::Matrix<typename DerivedA::Scalar, DerivedA::RowsAtCompileTime,
                  1>>::type
LinearSolve(const Eigen::MatrixBase<DerivedA>& A,
            const Eigen::MatrixBase<DerivedB>& b) {
  const LinearSolverType<Eigen::Matrix<double, DerivedA::RowsAtCompileTime,
                                       DerivedA::ColsAtCompileTime>>
      linear_solver(autoDiffToValueMatrix(A));
  return LinearSolve(linear_solver, A, b);
}
//@}

}  // namespace math
}  // namespace drake
