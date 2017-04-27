#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/dev/test/global_inverse_kinematics_test_util.h"
#include "drake/solvers/gurobi_solver.h"

using Eigen::Vector3d;
using Eigen::Isometry3d;

using drake::solvers::SolutionResult;

namespace drake {
namespace multibody {
namespace {
TEST_F(KukaTest, UnreachableTest) {
  // Test a cartesian pose that we know is not reachable.
  Eigen::Vector3d ee_pos_lb(0.6, -0.1, 0.7);
  Eigen::Vector3d ee_pos_ub(0.6, 0.1, 0.7);
  global_ik_.AddWorldPositionConstraint(ee_idx_, Vector3d::Zero(), ee_pos_lb,
                                        ee_pos_ub);

  Eigen::Quaterniond ee_desired_orient(
      Eigen::AngleAxisd(-M_PI, Vector3d(0, 1, 0)));
  global_ik_.AddWorldOrientationConstraint(ee_idx_, ee_desired_orient,
                                           0.0 * M_PI);

  solvers::GurobiSolver gurobi_solver;

  if (gurobi_solver.available()) {
    global_ik_.SetSolverOption(solvers::GurobiSolver::id(), "OutputFlag", 1);

    SolutionResult sol_result = gurobi_solver.Solve(global_ik_);

    EXPECT_TRUE(sol_result == SolutionResult::kInfeasible_Or_Unbounded ||
                sol_result == SolutionResult::kInfeasibleConstraints);
  }
  Eigen::Matrix<double, 7, 1> q_nom;
  Eigen::Matrix<double, 7, 1> q_guess;
  q_nom.setZero();
  q_guess.setZero();
  CheckNonlinearIK(ee_pos_lb, ee_pos_ub, ee_desired_orient, 0, q_nom, q_guess,
                   13);
}

TEST_F(KukaTest, ReachableWithCost) {
  // Test a reachable cartesian pose, test global IK with costs.
  // The cost is on the deviation to a desired posture q. Since q itself satisfy
  // the kinematics constraints we impose, the optimal solution should be q.
  const auto& joint_lb = rigid_body_tree_->joint_limit_min;
  const auto& joint_ub = rigid_body_tree_->joint_limit_max;
  DRAKE_DEMAND(rigid_body_tree_->get_num_positions() == 7);
  Eigen::Matrix<double, 7, 1> q = joint_lb;
  // Pick a posture within the joint bounds.
  for (int i = 0; i < 7; ++i) {
    q(i) += (joint_ub(i) - joint_lb(i)) * i / 10.0;
  }
  auto cache = rigid_body_tree_->CreateKinematicsCache();
  cache.initialize(q);
  rigid_body_tree_->doKinematics(cache);

  Isometry3d ee_desired_pose = rigid_body_tree_->CalcBodyPoseInWorldFrame(
      cache, rigid_body_tree_->get_body(ee_idx_));
  // Constrain the global IK to reach the exact end effector pose as the
  // posture q.
  global_ik_.AddWorldPositionConstraint(
      ee_idx_,  // body index
      Vector3d::Zero(),  // p_BQ
      ee_desired_pose.translation(),  // lower bound
      ee_desired_pose.translation());  // upper bound
  global_ik_.AddWorldOrientationConstraint(
      ee_idx_,  // body index
      Eigen::Quaterniond(ee_desired_pose.linear()),  // desired orientation
      0);  // tolerance.

  solvers::GurobiSolver gurobi_solver;

  if (gurobi_solver.available()) {
    // First solve the IK problem without the cost.
    global_ik_.SetSolverOption(solvers::GurobiSolver::id(), "OutputFlag", 1);
    SolutionResult sol_result = gurobi_solver.Solve(global_ik_);

    EXPECT_EQ(sol_result, SolutionResult::kSolutionFound);

    const Eigen::VectorXd q_no_cost =
        global_ik_.ReconstructGeneralizedPositionSolution();

    DRAKE_DEMAND(rigid_body_tree_->get_num_bodies() == 12);
    // Now add the cost on the posture error.
    // Any positive cost should be able to achieve the optimal solution
    // being equal to q.
    global_ik_.AddPostureCost(q, Eigen::VectorXd::Constant(12, 1),
                              Eigen::VectorXd::Constant(12, 1));

    sol_result = gurobi_solver.Solve(global_ik_);

    EXPECT_EQ(sol_result, SolutionResult::kSolutionFound);

    const Eigen::VectorXd q_w_cost =
        global_ik_.ReconstructGeneralizedPositionSolution();
    // There is extra error introduced from gurobi optimality condition and SVD,
    // so the tolerance is loose.
    EXPECT_TRUE(
        CompareMatrices(q_w_cost, q, 1E-2, MatrixCompareType::absolute));
    EXPECT_LE((q_w_cost - q).norm(), 1E-2);
    // The posture from IK with cost should be closer to q, than the posture
    // from IK without the cost.
    EXPECT_LE((q_w_cost - q).norm(), (q_no_cost - q).norm());
  }
}
}  // namespace
}  // namespace multibody
}  // namespace drake
