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
    // the obtuse-angled vertex. Here we constructed pts_[0] to be such vertex.
    // We verify that the angle at pts_[0] is larger than 120 degrees.
    if ((pts_[1] - pts_[0]).dot(pts_[2] - pts_[0]) /
            ((pts_[1] - pts_[0]).norm() * (pts_[2] - pts_[0]).norm()) >
        cos(M_PI / 3 * 2)) {
      throw std::runtime_error(
          "The angle at pts_[0] is less than 120 degrees, check whether pts_ "
          "is set correctly.");
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

ShortestDistanceFromPlaneToTwoPoints::ShortestDistanceFromPlaneToTwoPoints()
    : prog_{} {
  x_ = prog_.NewContinuousVariables<3>();
  // pts_[0] and pts_[1] are on the same side of the plane.
  pts_[0] = Eigen::Vector3d(1, 2, 3);
  pts_[1] = Eigen::Vector3d(-1, 3, 5);
  plane_normal_ = Eigen::Vector3d(2, 1, -2).normalized();
  plane_pt_ = Eigen::Vector3d(2, 1, -1);
  // clang-format off
  A_ << 1, 3, -2,
        2, 1,  4,
        0, 1,  0;
  // clang-format on
  prog_.AddL2NormCost(A_, -pts_[0], x_);
  prog_.AddL2NormCost(A_, -pts_[1], x_);
  prog_.AddLinearEqualityConstraint(plane_normal_.transpose() * A_,
                                    plane_normal_.dot(plane_pt_), x_);
}

void ShortestDistanceFromPlaneToTwoPoints::CheckSolution(
    const SolverInterface& solver,
    const std::optional<SolverOptions>& solver_options, double tol) const {
  MathematicalProgramResult result;
  if (solver.available() && solver.enabled()) {
    solver.Solve(prog_, std::nullopt, solver_options, &result);
    EXPECT_TRUE(result.is_success());
    // To compute the optimal solution in the closed form, we first compute y =
    // A_ * x. y should be on the plane plane_normal.dot(y) =
    // plane_normal.dot(plane_pt). It is the intersection between the plane and
    // the line passing through pt0 and the mirroring of pt1 across the plane.
    const Eigen::Vector3d pt1_mirror =
        pts_[1] - (pts_[1] - plane_pt_).dot(plane_normal_) * plane_normal_ * 2;
    const Eigen::Vector3d x_sol = result.GetSolution(x_);
    const Eigen::Vector3d y = A_ * x_sol;
    EXPECT_NEAR(plane_normal_.dot(y), plane_normal_.dot(plane_pt_), tol);
    EXPECT_TRUE(CompareMatrices((pt1_mirror - y).cross(y - pts_[0]),
                                Eigen::Vector3d::Zero(), tol));
    EXPECT_NEAR(result.get_optimal_cost(),
                (y - pts_[0]).norm() + (y - pts_[1]).norm(), tol);
  }
}
}  // namespace test
}  // namespace solvers
}  // namespace drake
