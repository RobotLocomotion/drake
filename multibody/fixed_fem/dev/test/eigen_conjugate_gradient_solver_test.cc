#include "drake/multibody/fixed_fem/dev/eigen_conjugate_gradient_solver.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/multibody/contact_solvers/sparse_linear_operator.h"

namespace drake {
namespace multibody {
namespace fixed_fem {
namespace internal {
namespace {
/* Dimension of the system. */
constexpr int kD = 4;
/* Tolerance for the CG solver. */
const double kTol = 1e-14;

/* Generate an arbitrary symmetric positive definite (SPD) matrix. */
Eigen::Matrix<double, kD, kD> MakeSpdMatrix() {
  /* Let B be an arbitrary full rank matrix. */
  Eigen::Matrix<double, kD, kD> B;
  // clang-format off
  B << 31.705908, 62.927869, 93.446683, 65.319711,
       12.200296, 32.078717, 13.422328, 25.142288,
       53.688842, 12.454311, 33.192683, 85.237341,
       14.656643, 42.836689, 13.362649, 85.326259;
  // clang-format on
  /* A = Bᵀ * B and is guaranteed to be symmetric positive definite. */
  return B.transpose() * B;
}

/* Generate an arbitrary right hand side vector b. */
Vector<double, kD> MakeRhs() {
  Vector<double, kD> b;
  b << 56.134, 43.548, 23.545, 12.374;
  return b;
}

/* Tests the solution of the linear solve is correct as measure by
 reconstruction errors. */
GTEST_TEST(EigenConjugateGradientSolverTest, DoubleSolverTest) {
  const Eigen::SparseMatrix<double> A = MakeSpdMatrix().sparseView();
  contact_solvers::internal::SparseLinearOperator<double> A_op("A", &A);
  const Vector<double, kD> b = MakeRhs();
  const EigenConjugateGradientSolver<double> solver(A_op, kTol);
  Eigen::Matrix<double, kD, 1> x;
  solver.Solve(b, &x);

  EXPECT_LE((A * x - b).norm(), b.norm() * kTol);
}

/* Tests the derivatives of the solution with respect to the right hand side
 matches the expected values. */
GTEST_TEST(EigenConjugateGradientSolverTest, AutoDiffSolverTest) {
  using T = AutoDiffXd;
  /* Build the operator A. */
  const Eigen::Matrix<double, kD, kD> A = MakeSpdMatrix();
  const Eigen::SparseMatrix<T> A_sparse = A.cast<T>().sparseView();
  contact_solvers::internal::SparseLinearOperator<T> A_op("A", &A_sparse);
  /* Build the right hand side b. */
  const Vector<double, kD> b = MakeRhs();
  Vector<T, kD> b_autodiff;
  math::initializeAutoDiff(b, b_autodiff);
  /* Build the solve and solve for x. */
  const EigenConjugateGradientSolver<T> solver(A_op, kTol);
  Eigen::Matrix<T, kD, 1> x;
  solver.Solve(b_autodiff, &x);
  /* Use LU decomposition to invert the matrix A. */
  const Eigen::FullPivLU<Eigen::Matrix<double, kD, kD>> lu(A);
  for (int i = 0; i < kD; ++i) {
    Vector<double, kD> unit_derivative = Vector<double, kD>::Zero();
    unit_derivative(i) = 1.0;
    const Vector<double, kD> expected_derivatives = lu.solve(unit_derivative);
    EXPECT_TRUE(CompareMatrices(x(i).derivatives(), expected_derivatives,
                                std::numeric_limits<double>::epsilon()));
  }
}
}  // namespace
}  // namespace internal
}  // namespace fixed_fem
}  // namespace multibody
}  // namespace drake
