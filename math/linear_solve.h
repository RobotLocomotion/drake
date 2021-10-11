#pragma once

#include <optional>

#include "drake/common/drake_deprecated.h"
#include "drake/common/symbolic.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace math {
namespace internal {

template <typename T>
using is_symbolic = std::is_same<T, symbolic::Expression>;

template <typename T>
inline constexpr bool is_symbolic_v = is_symbolic<T>::value;

template <typename T>
inline constexpr bool is_double_or_symbolic_v =
    std::disjunction<std::is_same<T, double>, is_symbolic<T>>::value;

template <typename T>
struct is_autodiff : std::false_type {};

template <int N>
struct is_autodiff<drake::AutoDiffd<N>> : std::true_type {};

template <typename T>
inline constexpr bool is_autodiff_v = is_autodiff<T>::value;
}  // namespace internal

/**
 * @anchor linear_solve_given_solver
 * @name solve linear system of equations with a given solver.
 * Solve linear system of equations A * x = b. Where A is an Eigen matrix of
 * double/AutoDiffScalar/symbolic::Expression, and b is an Eigen matrix of
 * double/AutoDiffScalar/symbolic::Expression.
 * Notice that if either A or b contains symbolic::Expression, then the other
 * has to contain symbolic::Expression.
 * This 3-argument version allows the user to re-use @p linear_solver when
 * @p b changes or the gradient of @p A changes. When either A or b contains
 * AutoDiffScalar, we use implicit function
 * theorem to find the gradient in x as ∂x/∂zᵢ = A⁻¹(∂b/∂zᵢ - ∂A/∂zᵢ * x) where
 * z is the variable we take gradient with.
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
 * A_ad.col(0) = InitializeAutoDiff(A_val.col(0), A0_gradient);
 * // Gradient of A.col(1)
 * Eigen::Matrix<double, 2, 3> A1_gradient;
 * A1_gradient << 7, 8, 9, 10, 11, 12;
 * A_ad.col(1) = InitializeAutoDiff(A_val.col(1), A1_gradient);
 * // Set the value and gradient of b to arbitrary value.
 * const Eigen::Vector2d b_val(2, 3);
 * Eigen::Matrix<double, 2, 3> b_gradient;
 * b_gradient << 1, 3, 5, 7, 9, 11;
 * const auto b_ad = InitializeAutoDiff(b_val, b_gradient);
 * // Solve the linear system A_val * x_val = b_val.
 * Eigen::PartialPivLU<Eigen::Matrix2d> linear_solver(A_val);
 * const auto x_val = SolveLinearSystem(linear_solver, A_val, b_val);
 * // Solve the linear system A*x=b, together with the gradient.
 * // x_ad contains both the value of the solution A*x=b, together with its
 * // gradient.
 * const auto x_ad = SolveLinearSystem(linear_solver, A_ad, b_ad);
 * @endcode
 */

//@{
/**
 * Specialized when A and b are both double or symbolic::Expression matrices.
 * See @ref linear_solve_given_solver for more details.
 * Note that @p A is unused, as we already compute its factorization in @p
 * linear_solver. But we keep it here for consistency with the overloaded
 * function, where A is a matrix of AutoDiffScalar.
 */
template <typename LinearSolver, typename DerivedA, typename DerivedB>
typename std::enable_if<
    internal::is_double_or_symbolic_v<typename DerivedA::Scalar> &&
        internal::is_double_or_symbolic_v<typename DerivedB::Scalar> &&
        std::is_same_v<typename DerivedA::Scalar, typename DerivedB::Scalar>,
    Eigen::Matrix<typename DerivedA::Scalar, DerivedA::RowsAtCompileTime,
                  DerivedB::ColsAtCompileTime>>::type
SolveLinearSystem(const LinearSolver& linear_solver,
                  const Eigen::MatrixBase<DerivedA>& A,
                  const Eigen::MatrixBase<DerivedB>& b) {
  unused(A);
  return linear_solver.solve(b);
}

/**
 * Specialized when the matrix in linear_solver and b are both double or
 * symbolic::Expression matrices.
 * See @ref linear_solve_given_solver for more details.
 */
template <typename LinearSolver, typename DerivedB>
typename std::enable_if<
    internal::is_double_or_symbolic_v<
        typename LinearSolver::MatrixType::Scalar> &&
        internal::is_double_or_symbolic_v<typename DerivedB::Scalar> &&
        std::is_same_v<typename LinearSolver::MatrixType::Scalar,
                       typename DerivedB::Scalar>,
    Eigen::Matrix<typename LinearSolver::MatrixType::Scalar,
                  DerivedB::RowsAtCompileTime,
                  DerivedB::ColsAtCompileTime>>::type
SolveLinearSystem(const LinearSolver& linear_solver,
                  const Eigen::MatrixBase<DerivedB>& b) {
  return linear_solver.solve(b);
}

template <typename LinearSolver, typename DerivedA, typename DerivedB>
DRAKE_DEPRECATED("2022-01-01",
                 "Please use SolveLinearSystem() instead of LinearSolve()")
typename std::enable_if<
    internal::is_double_or_symbolic_v<typename DerivedA::Scalar> &&
        internal::is_double_or_symbolic_v<typename DerivedB::Scalar> &&
        std::is_same_v<typename DerivedA::Scalar, typename DerivedB::Scalar>,
    Eigen::Matrix<typename DerivedA::Scalar, DerivedA::RowsAtCompileTime,
                  DerivedB::ColsAtCompileTime>>::type
    LinearSolve(const LinearSolver& linear_solver,
                const Eigen::MatrixBase<DerivedA>& A,
                const Eigen::MatrixBase<DerivedB>& b) {
  return SolveLinearSystem(linear_solver, A, b);
}

/**
 * Specialized the matrix in linear_solver is a double-valued matrix and b is
 * an AutoDiffScalar-valued matrix. See @ref linear_solve_given_solver for more
 * details.
 */
template <typename LinearSolver, typename DerivedB>
typename std::enable_if<
    std::is_same_v<typename LinearSolver::MatrixType::Scalar, double> &&
        internal::is_autodiff_v<typename DerivedB::Scalar>,
    Eigen::Matrix<typename DerivedB::Scalar, DerivedB::RowsAtCompileTime,
                  DerivedB::ColsAtCompileTime>>::type
SolveLinearSystem(const LinearSolver& linear_solver,
                  const Eigen::MatrixBase<DerivedB>& b) {
  const int num_z_b = GetDerivativeSize(b);
  if (num_z_b == 0) {
    return linear_solver.solve(ExtractValue(b))
        .template cast<typename DerivedB::Scalar>();
  }
  const Eigen::Matrix<double, DerivedB::RowsAtCompileTime,
                      DerivedB::ColsAtCompileTime>
      x_val = linear_solver.solve(ExtractValue(b));
  Eigen::Matrix<double, DerivedB::RowsAtCompileTime,
                DerivedB::Scalar::DerType::RowsAtCompileTime>
      grad(b.rows(), num_z_b);
  Eigen::Matrix<typename DerivedB::Scalar, DerivedB::RowsAtCompileTime,
                DerivedB::ColsAtCompileTime>
      x_ad(b.rows(), b.cols());
  for (int i = 0; i < b.cols(); ++i) {
    grad = ExtractGradient(b.col(i));
    // ∂x/∂zᵢ = A⁻¹*∂b/∂zᵢ
    // Before calling linear_solver.solve(grad.eval()), grad contains the
    // gradient ∂b/∂zᵢ. After calling this function grad contains ∂x/∂zᵢ. I call
    // grad.eval() to avoid aliasing issue.
    grad = linear_solver.solve(grad.eval());
    x_ad.col(i) = InitializeAutoDiff(x_val.col(i), grad);
  }
  return x_ad;
}

/**
 * Specialized when A is a double-valued matrix and b is an
 * AutoDiffScalar-valued matrix. See @ref linear_solve_given_solver for more
 * details. Note that @p A is unused, as we already compute its factorization in
 * @p linear_solver. But we keep it here for consistency with the overloaded
 * function, where A is a matrix of AutoDiffScalar.
 */
template <typename LinearSolver, typename DerivedA, typename DerivedB>
typename std::enable_if<
    std::is_same_v<typename DerivedA::Scalar, double> &&
        internal::is_autodiff_v<typename DerivedB::Scalar>,
    Eigen::Matrix<typename DerivedB::Scalar, DerivedA::RowsAtCompileTime,
                  DerivedB::ColsAtCompileTime>>::type
SolveLinearSystem(const LinearSolver& linear_solver,
                  const Eigen::MatrixBase<DerivedA>& A,
                  const Eigen::MatrixBase<DerivedB>& b) {
  unused(A);
  return SolveLinearSystem(linear_solver, b);
}

template <typename LinearSolver, typename DerivedA, typename DerivedB>
DRAKE_DEPRECATED("2022-01-01",
                 "Please use SolveLinearSystem() instead of LinearSolve()")
typename std::enable_if<
    std::is_same_v<typename DerivedA::Scalar, double> &&
        internal::is_autodiff_v<typename DerivedB::Scalar>,
    Eigen::Matrix<typename DerivedB::Scalar, DerivedA::RowsAtCompileTime,
                  DerivedB::ColsAtCompileTime>>::type
    LinearSolve(const LinearSolver& linear_solver,
                const Eigen::MatrixBase<DerivedA>& A,
                const Eigen::MatrixBase<DerivedB>& b) {
  return SolveLinearSystem(linear_solver, A, b);
}

/**
 * Specialized when A is an AutoDiffScalar-valued matrix, and b can contain
 * either AutoDiffScalar or double. See @ref linear_solve_given_solver for more
 * details.
 */
template <typename LinearSolver, typename DerivedA, typename DerivedB>
typename std::enable_if<
    internal::is_autodiff_v<typename DerivedA::Scalar>,
    Eigen::Matrix<typename DerivedA::Scalar, DerivedA::RowsAtCompileTime,
                  DerivedB::ColsAtCompileTime>>::type
SolveLinearSystem(const LinearSolver& linear_solver,
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
      return SolveLinearSystem(linear_solver, ExtractValue(A), b)
          .template cast<typename DerivedA::Scalar>();
    } else {
      return SolveLinearSystem(linear_solver, ExtractValue(A), ExtractValue(b))
          .template cast<typename DerivedA::Scalar>();
    }
  } else if (num_z_A == 0 && num_z_b != 0) {
    return SolveLinearSystem(linear_solver, ExtractValue(A), b);
  }
  // First compute the value of x.
  Eigen::Matrix<double, DerivedA::RowsAtCompileTime,
                DerivedB::ColsAtCompileTime>
      x_val;
  if constexpr (std::is_same_v<typename DerivedB::Scalar, double>) {
    x_val = linear_solver.solve(b);
  } else {
    const auto b_val = ExtractValue(b);
    x_val = linear_solver.solve(b_val);
  }

  // num_z_A != 0
  if (num_z_A != 0 && num_z_b != 0 && num_z_A != num_z_b) {
    throw std::runtime_error(
        fmt::format("SolveLinearSystem(): A contains derivatives for {} "
                    "variables, while b contains derivatives for {} variables",
                    num_z_A, num_z_b));
  }
  Eigen::Matrix<typename DerivedA::Scalar, DerivedA::RowsAtCompileTime,
                DerivedB::ColsAtCompileTime>
      x_ad(A.rows(), b.cols());
  // First sets the value of x_ad, and allocates the memory for the derivatives.
  // Note that I don't call InitializeAutoDiff since I have the
  // gradient of matrix x w.r.t scalar zᵢ, not the gradient of a vector x.col(j)
  // w.r.t a vector z.
  for (int i = 0; i < A.rows(); ++i) {
    for (int j = 0; j < b.cols(); ++j) {
      x_ad(i, j).value() = x_val(i, j);
      x_ad(i, j).derivatives() =
          Eigen::Matrix<double, DerivedA::Scalar::DerType::RowsAtCompileTime,
                        1>::Zero(num_z_A);
    }
  }

  Eigen::Matrix<double, DerivedA::RowsAtCompileTime,
                DerivedA::ColsAtCompileTime>
      dAdzi(A.rows(), A.cols());
  // This stores ∂b/∂zᵢ
  Eigen::Matrix<double, DerivedB::RowsAtCompileTime,
                DerivedB::ColsAtCompileTime>
      dbdzi(A.rows(), b.cols());
  // This stores ∂x/∂zᵢ
  Eigen::Matrix<double, DerivedA::RowsAtCompileTime,
                DerivedB::ColsAtCompileTime>
      dxdzi(A.rows(), b.cols());
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
    }
    if constexpr (!std::is_same_v<typename DerivedB::Scalar, double>) {
      for (int j = 0; j < b.rows(); ++j) {
        for (int k = 0; k < b.cols(); ++k) {
          if (b(j, k).derivatives().rows() != 0) {
            dbdzi(j, k) = b(j, k).derivatives()(i);
          }
        }
      }
    }
    dxdzi = linear_solver.solve(dbdzi - dAdzi * x_val);
    for (int j = 0; j < A.rows(); ++j) {
      for (int k = 0; k < b.cols(); ++k) {
        x_ad(j, k).derivatives()(i) = dxdzi(j, k);
      }
    }
  }
  return x_ad;
}

template <typename LinearSolver, typename DerivedA, typename DerivedB>
DRAKE_DEPRECATED("2022-01-01",
                 "Please use SolveLinearSystem() instead of LinearSolve()")
typename std::enable_if<
    internal::is_autodiff_v<typename DerivedA::Scalar>,
    Eigen::Matrix<typename DerivedA::Scalar, DerivedA::RowsAtCompileTime,
                  DerivedB::ColsAtCompileTime>>::type
    LinearSolve(const LinearSolver& linear_solver,
                const Eigen::MatrixBase<DerivedA>& A,
                const Eigen::MatrixBase<DerivedB>& b) {
  return SolveLinearSystem(linear_solver, A, b);
}

//@}

/**
 * @anchor get_linear_solver
 * @name Get linear solver
 *
 * Create the linear solver for a given matrix A, which will be used to solve
 * the linear system of equations A * x = b.
 *
 * The following table indicate the scalar type of the matrix in the returned
 * linear solver, depending on the scalar type in matrix A
 *
 * | A    | double |  ADS  | Expr |
 * |------|--------|-------|----- |
 * |solver| double | double| Expr |
 *
 * where ADS stands for Eigen::AutoDiffScalar, and Expr stands for
 * symbolic::Expression.
 * Here is the example code
 * @code{cc}
 * Eigen::Matrix2d A_val;
 * A_val << 1, 2, 2, 5;
 * Eigen::Vector2d b_val(3, 4);
 * const Eigen::Vector2d x_val =
 *   SolveLinearSystem(GetLinearSolver<Eigen::LLT>(A_val), A_val, b_val);
 * Eigen::Matrix<AutoDiffXd, 2, 2> A_ad;
 * A_ad(0, 0).value() = A_val(0, 0);
 * A_ad(0, 0).derivatives() = Eigen::Vector3d(1, 2, 3);
 * A_ad(0, 1).value() = A_val(0, 1);
 * A_ad(0, 1).derivatives() = Eigen::Vector3d(2, 3, 4);
 * A_ad(1, 0).value() = A_val(1, 0);
 * A_ad(1, 0).derivatives() = Eigen::Vector3d(3, 4, 5);
 * A_ad(1, 1).value() = A_val(1, 1);
 * A_ad(1, 1).derivatives() = Eigen::Vector3d(4, 5, 6);
 * // Solve A * x = b with A containing gradient.
 * const Eigen::Matrix<AutoDiffXd, 2, 1> x_ad1 =
 *   SolveLinearSystem(GetLinearSolver<Eigen::LLT>(A_ad), A_ad, b_val);
 * Eigen::Matrix<AutoDiffXd, 2, 1> b_ad;
 * b_ad(0).value() = b_val(0);
 * b_ad(0).derivatives() = Eigen::Vector3d(5, 6, 7);
 * b_ad(1).value() = b_val(1);
 * b_ad(1).derivatives() = Eigen::Vector3d(6, 7, 8);
 * // Solve A * x = b with b containing gradient.
 * const Eigen::Matrix<AutoDiffXd, 2, 1> x_ad2 =
 *   SolveLinearSystem(GetLinearSolver<Eigen::LLT>(A_val), A_val, b_ad);
 * // Solve A * x = b with both A and b containing gradient.
 * const Eigen::Matrix<AutoDiffXd, 2, 1> x_ad3 =
 *   SolveLinearSystem(GetLinearSolver<Eigen::LLT>(A_ad), A_ad, b_ad);
 * @endcode{cc}
 */

//@{

namespace internal {
/*
 * The return type of GetLinearSolver function. It is the type of the linear
 * solver. For example
 * LinearSolver<Eigen::LLT, Eigen::Matrix3d>::type is
 * Eigen::LLT<Eigen::Matrix3d>.
 * See @ref get_linear_solver for more details. When DerivedA::Scalar is
 * double or symbolic::Expression, the solver scalar type is the same as
 * DerivedA::Scalar; when DerivedA::Scalar is Eigen::AutoDiffScalar, the
 * solver scalar type is double.
 */
template <template <typename, int...> typename LinearSolverType,
          typename DerivedA>
using EigenLinearSolver = LinearSolverType<Eigen::Matrix<
    std::conditional_t<
        internal::is_double_or_symbolic_v<typename DerivedA::Scalar>,
        typename DerivedA::Scalar, double>,
    DerivedA::RowsAtCompileTime, DerivedA::ColsAtCompileTime, Eigen::ColMajor,
    DerivedA::MaxRowsAtCompileTime, DerivedA::MaxColsAtCompileTime>>;

/*
 * The most "promoted" of two scalar types; if they are the same, then that
 * type, if they are different and one is 'double', then whichever one is not
 * 'double'. Otherwise, the resulting type is 'void', indicating an error.
 */
template <typename ScalarA, typename ScalarB>
using Promoted = typename std::conditional_t<
    std::is_same_v<ScalarA, ScalarB>, ScalarA,
    std::conditional_t<
        std::is_same_v<double, ScalarA>, ScalarB,
        std::conditional_t<std::is_same_v<double, ScalarB>, ScalarA, void>>>;

/*
 * The type of the solution vector 'x' given DerivedA and DerivedB.
 */
template <typename DerivedA, typename DerivedB>
using Solution = typename Eigen::Matrix<
    Promoted<typename DerivedA::Scalar, typename DerivedB::Scalar>,
    DerivedA::RowsAtCompileTime, DerivedB::ColsAtCompileTime>;
}  // namespace internal

/**
 * Get the linear solver for a matrix A.
 * If A has scalar type of double or symbolic::Expressions, then the returned
 * linear solver will have the same scalar type. If A has scalar type of
 * Eigen::AutoDiffScalar, then the returned linear solver will have scalar type
 * of double. See @ref get_linear_solver for more details.
 */
template <template <typename, int...> typename LinearSolverType,
          typename DerivedA>
internal::EigenLinearSolver<LinearSolverType, DerivedA> GetLinearSolver(
    const Eigen::MatrixBase<DerivedA>& A) {
  if constexpr (internal::is_double_or_symbolic_v<typename DerivedA::Scalar>) {
    const internal::EigenLinearSolver<LinearSolverType, DerivedA> linear_solver(
        A);
    return linear_solver;
  } else {
    const auto A_val = ExtractValue(A);
    const internal::EigenLinearSolver<LinearSolverType, DerivedA> linear_solver(
        A_val);
    return linear_solver;
  }
}
//@}

/**
 * @anchor linear_solve
 * @name solve linear system of equations
 * Solve linear system of equations A * x = b. Where A is an Eigen matrix of
 * double/AutoDiffScalar/symbolic::Expression, and b is an Eigen matrix of
 * double/AutoDiffScalar/symbolic::Expression.
 * Note that when either A or b contains symbolic::Expression, the other has
 * to contain symbolic::Expression as well.
 * When either A or b contains AutoDiffScalar, we use implicit function theorem
 * to find the gradient in x as ∂x/∂zᵢ = A⁻¹(∂b/∂zᵢ - ∂A/∂zᵢ * x) where z is the
 * variable we take gradient with.
 *
 * The following table indicate the scalar type of x with A/b containing the
 * specified scalar type. The entries with NA are not supported.
 *
 * | b ＼ A | double | ADS | Expr |
 * |--------|--------|-----|----- |
 * | double | double | ADS |  NA  |
 * |   ADS  |   ADS  | ADS |  NA  |
 * |  Expr  |   NA   | NA  | Expr |
 *
 * where ADS stands for Eigen::AutoDiffScalar, and Expr stands for
 * symbolic::Expression.
 *
 * TODO(hongkai.dai): support one of A/b being a double matrix and the other
 * being a symbolic::Expression matrix.
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
 * A_ad.col(0) = InitializeAutoDiff(A_val.col(0), A0_gradient);
 * // Gradient of A.col(1)
 * Eigen::Matrix<double, 2, 3> A1_gradient;
 * A1_gradient << 7, 8, 9, 10, 11, 12;
 * A_ad.col(1) = InitializeAutoDiff(A_val.col(1), A1_gradient);
 * // Set the value and gradient of b to arbitrary value.
 * const Eigen::Vector2d b_val(2, 3);
 * Eigen::Matrix<double, 2, 3> b_gradient;
 * b_gradient << 1, 3, 5, 7, 9, 11;
 * const auto b_ad = InitializeAutoDiff(b_val, b_gradient);
 * // Solve the linear system A*x=b without the gradient.
 * const auto x_val = SolveLinearSystem<Eigen::PartialPivLU>(A_val, b_val);
 * // Solve the linear system A*x=b, together with the gradient.
 * // x_ad contains both the value of the solution A*x=b, together with its
 * // gradient.
 * const auto x_ad = SolveLinearSystem<Eigen::PartialPivLU>(A_ad, b_ad);
 * @endcode
 */

//@{
/**
 * Solves system A*x=b.  The supported combinations of scalar types are
 * summarized in the table above.  See @ref linear_solve for more details.
 */
template <template <typename, int...> typename LinearSolverType,
          typename DerivedA, typename DerivedB>
internal::Solution<DerivedA, DerivedB> SolveLinearSystem(
    const Eigen::MatrixBase<DerivedA>& A,
    const Eigen::MatrixBase<DerivedB>& b) {
  using ScalarA = typename DerivedA::Scalar;
  using ScalarB = typename DerivedB::Scalar;
  static_assert(
      std::is_same_v<ScalarA, ScalarB> || (!internal::is_symbolic_v<ScalarA> &&
                                           !internal::is_symbolic_v<ScalarB>),
      "Mixing symbolic and other types is not supported.");

  const auto linear_solver = GetLinearSolver<LinearSolverType>(A);
  return SolveLinearSystem(linear_solver, A, b);
}

template <template <typename, int...> typename LinearSolverType,
          typename DerivedA, typename DerivedB>
DRAKE_DEPRECATED("2022-01-01",
                 "Please use SolveLinearSystem() instead of LinearSolve()")
typename std::enable_if<
    internal::is_double_or_symbolic_v<typename DerivedA::Scalar> &&
        internal::is_double_or_symbolic_v<typename DerivedB::Scalar> &&
        std::is_same_v<typename DerivedA::Scalar, typename DerivedB::Scalar>,
    Eigen::Matrix<typename DerivedA::Scalar, DerivedA::RowsAtCompileTime,
                  DerivedB::ColsAtCompileTime>>::type
    LinearSolve(const Eigen::MatrixBase<DerivedA>& A,
                const Eigen::MatrixBase<DerivedB>& b) {
  return SolveLinearSystem<LinearSolverType>(A, b);
}

template <template <typename, int...> typename LinearSolverType,
          typename DerivedA, typename DerivedB>
DRAKE_DEPRECATED("2022-01-01",
                 "Please use SolveLinearSystem() instead of LinearSolve()")
typename std::enable_if<
    std::is_same_v<typename DerivedA::Scalar, double> &&
        internal::is_autodiff_v<typename DerivedB::Scalar>,
    Eigen::Matrix<typename DerivedB::Scalar, DerivedA::RowsAtCompileTime,
                  DerivedB::ColsAtCompileTime>>::type
    LinearSolve(const Eigen::MatrixBase<DerivedA>& A,
                const Eigen::MatrixBase<DerivedB>& b) {
  return SolveLinearSystem<LinearSolverType>(A, b);
}

template <template <typename, int...> typename LinearSolverType,
          typename DerivedA, typename DerivedB>
DRAKE_DEPRECATED("2022-01-01",
                 "Please use SolveLinearSystem() instead of LinearSolve()")
typename std::enable_if<
    internal::is_autodiff_v<typename DerivedA::Scalar>,
    Eigen::Matrix<typename DerivedA::Scalar, DerivedA::RowsAtCompileTime,
                  DerivedB::ColsAtCompileTime>>::type
    LinearSolve(const Eigen::MatrixBase<DerivedA>& A,
                const Eigen::MatrixBase<DerivedB>& b) {
  return SolveLinearSystem<LinearSolverType>(A, b);
}
//@}

/**
 * Solves a linear system of equations A*x=b.
 * Depending on the scalar types of A and b, the scalar type of x is summarized
 * in this table.
 * | b ＼ A | double | ADS | Expr |
 * |--------|--------|-----|----- |
 * | double | double | ADS |  NA  |
 * |   ADS  |   ADS  | ADS |  NA  |
 * |  Expr  |   NA   | NA  | Expr |
 *
 * where ADS stands for Eigen::AutoDiffScalar, and Expr stands for
 * symbolic::Expression.
 *
 * Using LinearSolver class is as fast as using Eigen's linear solver directly
 * when neither A nor b contains AutoDiffScalar. When either A or b contains
 * AutoDiffScalar, using LinearSolver is much faster than using Eigen's
 * autodiffable linear solver (for example
 * Eigen::LDLT<Eigen::Matrix<Eigen::AutoDiffScalar, 3, 3>>).
 *
 * Here is the example code
 * @code{cc}
 * Eigen::Matrix<AutoDiffXd, 2, 2> A;
 * A(0, 0).value() = 1;
 * A(0, 0).derivatives() = Eigen::Vector3d(1, 2, 3);
 * A(0, 1).value() = 2;
 * A(0, 1).derivatives() = Eigen::Vector3d(2, 3, 4);
 * A(1, 0).value() = 2;
 * A(1, 0).derivatives() = Eigen::Vector3d(3, 4, 5);
 * A(1, 1).value() = 5;
 * A(1, 1).derivatives() = Eigen::Vector3d(4, 5, 6);
 * LinearSolver<Eigen::LLT, Eigen::Matrix<AutoDiffXd, 2, 2>> solver(A);
 * Eigen::Matrix<AutoDiffXd, 2, 1> b;
 * b(0).value() = 2;
 * b(0).derivatives() = Eigen::Vector3d(1, 2, 3);
 * b(1).value() = 3;
 * b(1).derivatives() = Eigen::Vector3d(4, 5, 6);
 * Eigen::Matrix<AutoDiffXd, 2, 1> x = solver.Solve(b);
 * @endcode
 *
 */
template <template <typename, int...> typename LinearSolverType,
          typename DerivedA>
class LinearSolver {
 public:
  using SolverType = internal::EigenLinearSolver<LinearSolverType, DerivedA>;

  /**
   * The return type of Solve() function.
   * When both A and B contain the same scalar, and that scalar type
   * is double or symbolic::Expression, then the return type is
   * Eigen::Solve<Decomposition, DerivedB>, same as the return type of
   * Decomposition::solve() function in Eigen. This avoids unnecessary copies
   * and heap memory allocations if we were to evaluate
   * Eigen::Solve<Decomposition, DerivedB> to a concrete Eigen::Matrix type.
   * Othewise we return an Eigen::Matrix with the same size as DerivedB
   * and proper scalar type.
   */
  template <typename DerivedB>
  using SolutionType = std::conditional_t<
      std::is_same_v<typename DerivedA::Scalar, typename DerivedB::Scalar> &&
          internal::is_double_or_symbolic_v<typename DerivedA::Scalar>,
      Eigen::Solve<SolverType, DerivedB>,
      internal::Solution<DerivedA, DerivedB>>;

  /** Default constructor. Constructs an empty linear solver. */
  LinearSolver() : eigen_linear_solver_() {}

  explicit LinearSolver(const Eigen::MatrixBase<DerivedA>& A)
      : eigen_linear_solver_{GetLinearSolver<LinearSolverType>(A)} {
    if constexpr (internal::is_autodiff_v<typename DerivedA::Scalar>) {
      A_.emplace(A);
    }
  }

  /**
   * Solves system A*x = b.
   * Return type is as described in the table above.
   */
  template <typename DerivedB>
  SolutionType<DerivedB> Solve(const Eigen::MatrixBase<DerivedB>& b) const {
    using ScalarA = typename DerivedA::Scalar;
    using ScalarB = typename DerivedB::Scalar;
    static_assert(std::is_same_v<ScalarA, ScalarB> ||
                      (!internal::is_symbolic_v<ScalarA> &&
                       !internal::is_symbolic_v<ScalarB>),
                  "Mixing symbolic and other types is not supported.");

    if constexpr (std::is_same_v<ScalarA, ScalarB> &&
                  !internal::is_autodiff_v<ScalarA>) {
      return eigen_linear_solver_.solve(b);
      // NOLINTNEXTLINE(readability/braces)
    } else if constexpr (std::is_same_v<ScalarA, double> &&
                         internal::is_autodiff_v<ScalarB>) {
      return SolveLinearSystem(eigen_linear_solver_, b);
      // NOLINTNEXTLINE(readability/braces)
    } else if constexpr (internal::is_autodiff_v<ScalarA>) {
      return SolveLinearSystem(eigen_linear_solver_, *A_, b);
    }
    DRAKE_UNREACHABLE();
  }

  /** Getter for the Eigen linear solver.
   * The scalar type in the Eigen linear solver depends on the scalar type in A
   * matrix, as shown in this table
   * |      A       | double |  ADS   | Expr |
   * |--------------|--------|--------|----- |
   * |linear_solver | double | double | Expr |
   *
   * where ADS stands for Eigen::AutoDiffScalar, Expr stands for
   * symbolic::Expression.
   *
   * Note that when A contains autodiffscalar, we only use the double version of
   * Eigen linear solver. By using implicit-function theorem with the
   * double-valued Eigen linear solver, we can compute the gradient of the
   * solution much faster than directly autodiffing the Eigen linear solver.
   *
   */
  const SolverType& eigen_linear_solver() const { return eigen_linear_solver_; }

 private:
  SolverType eigen_linear_solver_;
  std::optional<Eigen::Matrix<
      typename DerivedA::Scalar, DerivedA::RowsAtCompileTime,
      DerivedA::ColsAtCompileTime, Eigen::ColMajor,
      DerivedA::MaxRowsAtCompileTime, DerivedA::MaxColsAtCompileTime>>
      A_;
};

}  // namespace math
}  // namespace drake
