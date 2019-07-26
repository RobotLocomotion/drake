#include "drake/systems/trajectory_optimization/integration_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace systems {
namespace trajectory_optimization {
GTEST_TEST(MidPointIntegrationConstraintTest, Test) {
  MidPointIntegrationConstraint constraint(3);
  EXPECT_EQ(constraint.num_constraints(), 3);
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
}
}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace drake
