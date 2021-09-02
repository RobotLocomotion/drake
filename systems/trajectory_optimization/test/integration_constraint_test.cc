#include "drake/systems/trajectory_optimization/integration_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
GTEST_TEST(MidPointIntegrationConstraintTest, Test) {
  const int dim{3};
  MidPointIntegrationConstraint constraint(dim);
  EXPECT_EQ(constraint.num_constraints(), dim);
  EXPECT_EQ(constraint.num_vars(), 13);
  // Evaluate the constraint with arbitrary input.
  const Eigen::Vector3d x_r(2, 3, 4);
  const Eigen::Vector3d x_l(2.5, 3.4, 4.1);
  const Eigen::Vector3d xdot_r(0.4, 1.5, -2.7);
  const Eigen::Vector3d xdot_l(3.4, 1.1, -1.3);
  const double dt = 0.5;
  Eigen::VectorXd constraint_x;
  constraint.ComposeX<double>(x_r, x_l, xdot_r, xdot_l, dt, &constraint_x);
  Eigen::VectorXd constraint_y;
  constraint.Eval(constraint_x, &constraint_y);
  const Eigen::Vector3d y_expected = x_r - x_l - (xdot_r + xdot_l) / 2 * dt;
  EXPECT_TRUE(CompareMatrices(constraint_y, y_expected, 1E-14));

  // Now check the gradient sparsity pattern.
  const auto x_autodiff = math::InitializeAutoDiff(
      constraint_x,
      Eigen::MatrixXd::Identity(constraint_x.rows(), constraint_x.rows()));

  AutoDiffVecXd y_autodiff;
  constraint.Eval(x_autodiff, &y_autodiff);
  const Eigen::MatrixXd y_grad = math::ExtractGradient(y_autodiff);
  // y_grad_sparse only contains the non-zero entries in y_grad. The non-zero
  // entries are stored in constraint.gradient_sparsity_pattern().
  Eigen::MatrixXd y_grad_sparse(constraint.num_constraints(),
                                constraint.num_vars());
  y_grad_sparse.setZero();
  for (const auto& item : constraint.gradient_sparsity_pattern().value()) {
    y_grad_sparse(item.first, item.second) = y_grad(item.first, item.second);
  }
  EXPECT_TRUE(CompareMatrices(y_grad, y_grad_sparse));

  // Compute gradient by hand.
  Eigen::MatrixXd y_grad_expected(constraint.num_constraints(),
                                  constraint.num_vars());
  y_grad_expected.block(0, 0, dim, dim) = Eigen::MatrixXd::Identity(dim, dim);
  y_grad_expected.block(0, dim, dim, dim) =
      -Eigen::MatrixXd::Identity(dim, dim);
  y_grad_expected.block(0, 2 * dim, dim, dim) =
      -dt / 2 * Eigen::MatrixXd::Identity(dim, dim);
  y_grad_expected.block(0, 3 * dim, dim, dim) =
      -dt / 2 * Eigen::MatrixXd::Identity(dim, dim);
  y_grad_expected.col(4 * dim) = -(xdot_r + xdot_l) / 2;
  EXPECT_TRUE(CompareMatrices(y_grad, y_grad_expected, 1E-14));
}
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
