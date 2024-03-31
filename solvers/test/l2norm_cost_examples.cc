#include "drake/solvers/test/l2norm_cost_examples.h"

#include "drake/common/test_utilities/eigen_matrix_compare.h"
namespace drake {
namespace solvers {
namespace test {
ShortestDistanceToThreePoints::ShortestDistanceToThreePoints() : prog_{} {
  x_ = prog_.NewContinuousVariables<3>();
  pts_[0] = Eigen::Vector3d(1, 2, 3);
  pts_[1] = Eigen::Vector3d(4, 5, 6);
  pts_[2] = Eigen::Vector3d(4, -1, -9);
  for (int i = 0; i < 3; ++i) {
    prog_.AddL2NormCost(Eigen::Matrix3d::Identity(), -pts_[i], x_);
  }
}

void ShortestDistanceToThreePoints::CheckSolution(
    const SolverInterface& solver,
    const std::optional<SolverOptions>& solver_options, double tol) const {
  MathematicalProgramResult result;
  if (solver.available() && solver.enabled()) {
    solver.Solve(prog_, std::nullopt, solver_options, &result);
    EXPECT_TRUE(result.is_success());
    const Eigen::Vector3d x_sol = result.GetSolution(x_);
    // The solution of x is the Fermat point of the triangle. If the triangle
    // has an angle greater than 120 degrees, then the Fermat point is sited at
    // the obtuse-angled vertex. Here we construected pts_[0] to be such vertex.
    // We verify that the angle at pts_[0] is larger than 120 degrees.
    if ((pts_[1] - pts_[0]).dot(pts_[2] - pts_[0]) /
            ((pts_[1] - pts_[0]).norm() * (pts_[2] - pts_[0]).norm()) >
        cos(M_PI / 3 * 2)) {
      throw std::runtime_error(
          "The angle at pts_[0] is less than 120 degrees.");
    }
    const Eigen::Vector3d x_sol_expected = pts_[0];
    EXPECT_TRUE(CompareMatrices(x_sol, x_sol_expected, tol));
    EXPECT_NEAR(result.get_optimal_cost(),
                (pts_[0] - x_sol_expected).norm() +
                    (pts_[1] - x_sol_expected).norm() +
                    (pts_[2] - x_sol_expected).norm(),
                tol);
  }
}

ShortestDistanceFromCylinderToPoint::ShortestDistanceFromCylinderToPoint()
    : prog_{} {
  x_ = prog_.NewContinuousVariables<3>();
  prog_.AddBoundingBoxConstraint(-1, 1, x_[2]);
  prog_.AddLorentzConeConstraint(
      Vector3<symbolic::Expression>(2, x_[0] * 1, x_[1] * 1));
  pt_ = Eigen::Vector3d(1, 2, 5);
  prog_.AddL2NormCost(Eigen::Matrix3d::Identity(), -pt_, x_);
}

void ShortestDistanceFromCylinderToPoint::CheckSolution(
    const SolverInterface& solver,
    const std::optional<SolverOptions>& solver_options, double tol) const {
  MathematicalProgramResult result;
  if (solver.available() && solver.enabled()) {
    solver.Solve(prog_, std::nullopt, solver_options, &result);
    const Eigen::Vector3d x_sol = result.GetSolution(x_);
    const Eigen::Vector3d x_sol_expected(2 / std::sqrt(5), 4 / std::sqrt(5), 1);
    EXPECT_TRUE(CompareMatrices(x_sol, x_sol_expected, tol));
    EXPECT_NEAR(result.get_optimal_cost(), (x_sol_expected - pt_).norm(), tol);
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
