#include "drake/multibody/inverse_kinematics/test/global_inverse_kinematics_test_util.h"
#include "drake/solvers/gurobi_solver.h"

using Eigen::Isometry3d;
using Eigen::Vector3d;

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
    global_ik_.get_mutable_prog()->SetSolverOption(solvers::GurobiSolver::id(),
                                                   "OutputFlag", 1);

    solvers::MathematicalProgramResult result =
        gurobi_solver.Solve(global_ik_.prog());

    EXPECT_TRUE(result.is_success());

    double pos_tol = 0.061;
    double orient_tol = 0.24;
    CheckGlobalIKSolution(result, pos_tol, orient_tol);
    // Now call nonlinear IK with the solution from global IK as the initial
    // seed. If the global IK provides a good initial seed, then the nonlinear
    // IK should be able to find a solution.
    Eigen::Matrix<double, 7, 1> q_global_ik =
        global_ik_.ReconstructGeneralizedPositionSolution(result);
    CheckNonlinearIK(ee_pos_lb_W, ee_pos_ub_W, ee_desired_orient, angle_tol,
                     q_global_ik, q_global_ik, 1);

    // Now update the constraint, the problem should still be feasible.
    // TODO(hongkai.dai): do a warm start on the binary variables
    ee_pos_lb_W << 0.2, -0.1, 0.6;
    ee_pos_ub_W << 0.4, 0.1, 1.0;
    ee_pos_cnstr.evaluator()->UpdateLowerBound(ee_pos_lb_W);
    ee_pos_cnstr.evaluator()->UpdateUpperBound(ee_pos_ub_W);
    angle_tol = 0.3 * M_PI;
    // The orientation constraint is 2 * cos(angle_tol) + 1 <= trace(Rᵀ * R_des)
    ee_orient_cnstr.evaluator()->UpdateLowerBound(
        Vector1d(2 * cos(angle_tol) + 1));
    result = gurobi_solver.Solve(global_ik_.prog());
    EXPECT_TRUE(result.is_success());
    const auto& ee_pos_sol =
        result.GetSolution(global_ik_.body_position(ee_idx_));
    EXPECT_TRUE((ee_pos_sol.array() >= ee_pos_lb_W.array() - 1E-6).all());
    EXPECT_TRUE((ee_pos_sol.array() <= ee_pos_ub_W.array() + 1E-6).all());
    const auto& ee_rotmat_sol =
        result.GetSolution(global_ik_.body_rotation_matrix(ee_idx_));
    Eigen::AngleAxisd ee_orient_err(ee_rotmat_sol.transpose() *
                                    ee_desired_orient.toRotationMatrix());
    EXPECT_LE(ee_orient_err.angle(), angle_tol + 1E-4);

    q_global_ik = global_ik_.ReconstructGeneralizedPositionSolution(result);
    CheckNonlinearIK(ee_pos_lb_W, ee_pos_ub_W, ee_desired_orient, angle_tol,
                     q_global_ik, q_global_ik, 1);

    // Now tighten the joint limits, the problem should be feasible.
    const Body<double>& iiwa_link_2 = plant_->GetBodyByName("iiwa_link_2");
    const Body<double>& iiwa_link_3 = plant_->GetBodyByName("iiwa_link_3");
    global_ik_.AddJointLimitConstraint(iiwa_link_2.index(), 0.5, 0.5);
    global_ik_.AddJointLimitConstraint(iiwa_link_3.index(), -0.5, -0.4);
    result = gurobi_solver.Solve(global_ik_.prog());
    EXPECT_TRUE(result.is_success());
    q_global_ik = global_ik_.ReconstructGeneralizedPositionSolution(result);
    // The reconstructed posture should be within the user specified bound.
    EXPECT_NEAR(q_global_ik(
                    int{plant_->GetJointByName("iiwa_joint_2").index()}),
                0.5, 1e-3);
    const JointIndex iiwa_joint_3_index =
        plant_->GetJointByName("iiwa_joint_3").index();
    EXPECT_GE(q_global_ik(int{iiwa_joint_3_index}), -0.5);
    EXPECT_LE(q_global_ik(int{iiwa_joint_3_index}), -0.4);

    // Now further tighten the joint limits, the problem should be infeasible
    global_ik_.AddJointLimitConstraint(
        plant_->GetBodyByName("iiwa_link_4").index(), 0.5, 0.55);
    result = gurobi_solver.Solve(global_ik_.prog());
    EXPECT_TRUE(result.get_solution_result() ==
                    solvers::SolutionResult::kInfeasibleConstraints ||
                result.get_solution_result() ==
                    solvers::SolutionResult::kInfeasible_Or_Unbounded);
  }
}
}  // namespace
}  // namespace multibody
}  // namespace drake
