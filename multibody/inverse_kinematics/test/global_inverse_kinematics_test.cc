#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/inverse_kinematics/test/global_inverse_kinematics_test_util.h"
#include "drake/solvers/gurobi_solver.h"

using Eigen::Isometry3d;
using Eigen::Vector3d;

using drake::solvers::SolutionResult;

namespace drake {
namespace multibody {
namespace {
GTEST_TEST(GlobalInverseKinematicsTest, TestConstructor) {
  // Test the constructor. With options linear_constraint_only_ = false, the
  // program contains lorentz cone constraint. Otherwise there are only linear
  // constraints.
  auto kuka = ConstructKuka();
  GlobalInverseKinematics::Options global_ik_options;
  // The default option sets linear_constraint_only_ to false.
  GlobalInverseKinematics global_ik_default(*kuka, global_ik_options);
  EXPECT_FALSE(global_ik_default.prog().lorentz_cone_constraints().empty());
  EXPECT_FALSE(
      global_ik_default.prog().rotated_lorentz_cone_constraints().empty());

  global_ik_options.linear_constraint_only = true;
  GlobalInverseKinematics global_ik_milp(*kuka, global_ik_options);
  EXPECT_TRUE(global_ik_milp.prog().lorentz_cone_constraints().empty());
  EXPECT_TRUE(global_ik_milp.prog().rotated_lorentz_cone_constraints().empty());
}

TEST_F(KukaTest, UnreachableTest) {
  // Test a cartesian pose that we know is not reachable.
  Eigen::Vector3d ee_pos_lb(0.8, -0.1, 0.7);
  Eigen::Vector3d ee_pos_ub(0.8, 0.1, 0.7);
  global_ik_.AddWorldPositionConstraint(ee_idx_, Vector3d::Zero(), ee_pos_lb,
                                        ee_pos_ub);

  Eigen::Quaterniond ee_desired_orient(
      Eigen::AngleAxisd(-M_PI, Vector3d(0, 1, 0)));
  global_ik_.AddWorldOrientationConstraint(ee_idx_, ee_desired_orient,
                                           0.0 * M_PI);

  solvers::GurobiSolver gurobi_solver;
  if (gurobi_solver.available()) {
    global_ik_.get_mutable_prog()->SetSolverOption(solvers::GurobiSolver::id(),
                                                   "OutputFlag", 1);

    const solvers::MathematicalProgramResult result =
        gurobi_solver.Solve(global_ik_.prog(), {}, {});
    EXPECT_TRUE(result.get_solution_result() ==
                    SolutionResult::kInfeasible_Or_Unbounded ||
                result.get_solution_result() ==
                    SolutionResult::kInfeasibleConstraints);
  }
  Eigen::Matrix<double, 7, 1> q_nom;
  Eigen::Matrix<double, 7, 1> q_guess;
  q_nom.setZero();
  q_guess.setZero();
  CheckNonlinearIK(ee_pos_lb, ee_pos_ub, ee_desired_orient, 0, q_nom, q_guess,
                   false);
}

TEST_F(KukaTest, ReachableWithCost) {
  // Test a reachable cartesian pose, test global IK with costs.
  // The cost is on the deviation to a desired posture q. Since q itself satisfy
  // the kinematics constraints we impose, the optimal solution should be q.
  const auto& joint_lb = plant_->GetPositionLowerLimits();
  const auto& joint_ub = plant_->GetPositionUpperLimits();
  DRAKE_DEMAND(plant_->num_positions() == 7);
  Eigen::Matrix<double, 7, 1> q = joint_lb;
  // Pick a posture within the joint bounds.
  for (int i = 0; i < 7; ++i) {
    q(i) += (joint_ub(i) - joint_lb(i)) * i / 10.0;
  }
  auto context = plant_->CreateDefaultContext();
  plant_->SetPositions(context.get(), q);

  math::RigidTransformd ee_desired_pose = plant_->CalcRelativeTransform(
      *context, plant_->world_frame(),
      plant_->get_body(BodyIndex{ee_idx_}).body_frame());
  // Constrain the global IK to reach the exact end effector pose as the
  // posture q.
  global_ik_.AddWorldPositionConstraint(
      ee_idx_,                         // body index
      Vector3d::Zero(),                // p_BQ
      ee_desired_pose.translation(),   // lower bound
      ee_desired_pose.translation());  // upper bound
  global_ik_.AddWorldOrientationConstraint(
      ee_idx_,  // body index
      Eigen::Quaterniond(
          ee_desired_pose.rotation().matrix()),  // desired orientation
      0);                                        // tolerance.

  solvers::GurobiSolver gurobi_solver;

  if (gurobi_solver.available()) {
    // First solve the IK problem without the cost.
    global_ik_.get_mutable_prog()->SetSolverOption(solvers::GurobiSolver::id(),
                                                   "OutputFlag", 1);

    solvers::MathematicalProgramResult result =
        gurobi_solver.Solve(global_ik_.prog(), {}, {});
    EXPECT_TRUE(result.is_success());

    const Eigen::VectorXd q_no_cost =
        global_ik_.ReconstructGeneralizedPositionSolution(result);

    // Now add the cost on the posture error.
    // Any positive cost should be able to achieve the optimal solution
    // being equal to q.
    global_ik_.AddPostureCost(
        q, Eigen::VectorXd::Constant(plant_->num_bodies(), 1),
        Eigen::VectorXd::Constant(plant_->num_bodies(), 1));

    result = gurobi_solver.Solve(global_ik_.prog(), {}, {});
    EXPECT_TRUE(result.is_success());

    // The position tolerance and the orientation tolerance is chosen
    // according to the Gurobi solver tolerance.
    double position_error = 1E-5;
    double orientation_error = 2E-5;
    CheckGlobalIKSolution(result, position_error, orientation_error);

    const Eigen::VectorXd q_w_cost =
        global_ik_.ReconstructGeneralizedPositionSolution(result);
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
