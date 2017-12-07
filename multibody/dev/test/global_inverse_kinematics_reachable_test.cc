#include "drake/multibody/dev/test/global_inverse_kinematics_test_util.h"
#include "drake/solvers/gurobi_solver.h"

using Eigen::Vector3d;
using Eigen::Isometry3d;

using drake::solvers::SolutionResult;

namespace drake {
namespace multibody {
namespace {
TEST_F(KukaTest, ReachableTest) {
  // Test the case that global IK should find a solution.
  // "ee" stands for "end effector".
  Eigen::Vector3d ee_pos_lb_W(0.4, -0.1, 0.4);
  Eigen::Vector3d ee_pos_ub_W(0.6, 0.1, 0.6);
  auto ee_pos_cnstr = global_ik_.AddWorldPositionConstraint(
      ee_idx_, Vector3d::Zero(), ee_pos_lb_W, ee_pos_ub_W);

  Eigen::Quaterniond ee_desired_orient(
      Eigen::AngleAxisd(-M_PI / 2, Vector3d(0, 1, 0)));
  double angle_tol = 0.2 * M_PI;
  auto ee_orient_cnstr = global_ik_.AddWorldOrientationConstraint(
      ee_idx_, ee_desired_orient, angle_tol);

  solvers::GurobiSolver gurobi_solver;
  if (gurobi_solver.available()) {
    global_ik_.SetSolverOption(solvers::GurobiSolver::id(), "OutputFlag", 1);

    SolutionResult sol_result = gurobi_solver.Solve(global_ik_);

    EXPECT_EQ(sol_result, SolutionResult::kSolutionFound);

    double pos_tol = 0.06;
    double orient_tol = 0.24;
    CheckGlobalIKSolution(pos_tol, orient_tol);
    // Now call nonlinear IK with the solution from global IK as the initial
    // seed. If the global IK provides a good initial seed, then the nonlinear
    // IK should be able to find a solution.
    Eigen::Matrix<double, 7, 1> q_global_ik =
        global_ik_.ReconstructGeneralizedPositionSolution();
    CheckNonlinearIK(ee_pos_lb_W, ee_pos_ub_W,
                     ee_desired_orient, angle_tol, q_global_ik, q_global_ik, 1);

    // Now update the constraint, the problem should still be feasible.
    // TODO(hongkai.dai): do a warm start on the binary variables
    ee_pos_lb_W << 0.2, -0.1, 0.6;
    ee_pos_ub_W << 0.4, 0.1, 1.0;
    ee_pos_cnstr.constraint()->UpdateLowerBound(ee_pos_lb_W);
    ee_pos_cnstr.constraint()->UpdateUpperBound(ee_pos_ub_W);
    angle_tol = 0.3 * M_PI;
    // The orientation constraint is 2 * cos(angle_tol) + 1 <= trace(Rᵀ * R_des)
    ee_orient_cnstr.constraint()->UpdateLowerBound(
        Vector1d(2 * cos(angle_tol) + 1));
    sol_result = gurobi_solver.Solve(global_ik_);
    EXPECT_EQ(sol_result, SolutionResult::kSolutionFound);
    const auto& ee_pos_sol =
        global_ik_.GetSolution(global_ik_.body_position(ee_idx_));
    EXPECT_TRUE((ee_pos_sol.array() >= ee_pos_lb_W.array() - 1E-6).all());
    EXPECT_TRUE((ee_pos_sol.array() <= ee_pos_ub_W.array() + 1E-6).all());
    const auto& ee_rotmat_sol =
        global_ik_.GetSolution(global_ik_.body_rotation_matrix(ee_idx_));
    Eigen::AngleAxisd ee_orient_err(ee_rotmat_sol.transpose() *
                                    ee_desired_orient.toRotationMatrix());
    EXPECT_LE(ee_orient_err.angle(), angle_tol + 1E-4);

    q_global_ik = global_ik_.ReconstructGeneralizedPositionSolution();
    CheckNonlinearIK(ee_pos_lb_W, ee_pos_ub_W,
                     ee_desired_orient, angle_tol, q_global_ik, q_global_ik, 1);

    // Now tighten the joint limits, the problem should be feasible.
    global_ik_.AddJointLimitConstraint(3, 0.5, 0.5);
    global_ik_.AddJointLimitConstraint(4, -0.5, -0.4);
    sol_result = gurobi_solver.Solve(global_ik_);
    EXPECT_EQ(sol_result, solvers::SolutionResult::kSolutionFound);
    q_global_ik = global_ik_.ReconstructGeneralizedPositionSolution();
    // The reconstructed posture should be within the user specified bound.
    EXPECT_NEAR(q_global_ik(0), 0.5, 1e-3);
    EXPECT_GE(q_global_ik(1), -0.5);
    EXPECT_LE(q_global_ik(1), -0.4);

    // Now further tighten the joint limits, the problem should be infeasible
    global_ik_.AddJointLimitConstraint(5, 0.5, 0.55);
    sol_result = gurobi_solver.Solve(global_ik_);
    EXPECT_TRUE(sol_result == solvers::SolutionResult::kInfeasibleConstraints ||
                sol_result ==
                    solvers::SolutionResult::kInfeasible_Or_Unbounded);
    q_global_ik = global_ik_.ReconstructGeneralizedPositionSolution();
  }
}
}  // namespace
}  // namespace multibody
}  // namespace drake
