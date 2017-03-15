#include "drake/multibody/global_inverse_kinematics.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_matrix_compare.h"
#include "drake/common/drake_path.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/solvers/gurobi_solver.h"

using Eigen::Vector3d;
using Eigen::Isometry3d;

using drake::solvers::SolutionResult;

namespace drake {
namespace multibody {
namespace {
std::unique_ptr<RigidBodyTree<double>> ConstructKuka() {
  std::unique_ptr<RigidBodyTree<double>> rigid_body_tree =
      std::make_unique<RigidBodyTree<double>>();
  const std::string model_path = drake::GetDrakePath() +
                                 "/examples/kuka_iiwa_arm/models/iiwa14/"
                                 "iiwa14_simplified_collision.urdf";

  parsers::urdf::AddModelInstanceFromUrdfFile(
      model_path,
      drake::multibody::joints::kFixed,
      nullptr,
      rigid_body_tree.get());

  AddFlatTerrainToWorld(rigid_body_tree.get());
  return rigid_body_tree;
}

class KukaTest : public ::testing::Test {
 public:
  KukaTest()
      : rigid_body_tree_(ConstructKuka()),
        global_ik_(*rigid_body_tree_, 2),  // Test with 2 binary variables per
                                           // half axis.
        ee_idx_(rigid_body_tree_->FindBodyIndex("iiwa_link_ee")) {}

  ~KukaTest() override {};

  /**
   * Given the solution computed from global IK, reconstruct the posture, then
   * compute the body pose using forward kinematics with the reconstructed
   * posture. Compare that forward kinematics body pose, with the body pose
   * in the global IK.
   */
  void CheckGlobalIKSolution(double pos_tol, double orient_tol) const {
    Eigen::VectorXd q_global_ik =
        global_ik_.ReconstructGeneralizedPositionSolution();

    std::cout << "Reconstructed robot posture:\n" << q_global_ik << std::endl;
    KinematicsCache<double> cache = rigid_body_tree_->doKinematics(q_global_ik);

    // TODO(hongkai.dai): replace this print out with error check. We have not
    // derived a rigorous bound on the rotation matrix relaxation yet. Should be
    // able to get a more meaningful bound when we have some theoretical proof.
    for (int i = 1; i < rigid_body_tree_->get_num_bodies(); ++i) {
      // Compute forward kinematics.
      const auto &body_pose_fk = rigid_body_tree_->CalcFramePoseInWorldFrame(
          cache, rigid_body_tree_->get_body(i), Isometry3d::Identity());

      const Eigen::Matrix3d body_Ri =
          global_ik_.GetSolution(global_ik_.body_rotation_matrix(i));
      EXPECT_TRUE((body_Ri.array().abs() <= 1).all());
      EXPECT_LE(body_Ri.trace(), 3);
      EXPECT_GE(body_Ri.trace(), -1);
      // TODO(hongkai.dai): We will have a more meaningful bound on the
      // relaxation of rotation matrix. Then clean up this print out with
      // the check on the error bound, and move this file out of dev folder.
      std::cout << rigid_body_tree_->get_body(i).get_name() << std::endl;
      std::cout << "rotation matrix:\n global_ik\n" << body_Ri << std::endl;
      std::cout << "forward kinematics\n" << body_pose_fk.linear() << std::endl;
      std::cout << "R * R':\n" << body_Ri * body_Ri.transpose() << std::endl;
      std::cout << "det(R) = " << body_Ri.determinant() << std::endl;
      Vector3d body_pos_global_ik =
          global_ik_.GetSolution(global_ik_.body_position(i));
      std::cout << "position:\n global_ik\n" << body_pos_global_ik << std::endl;
      std::cout << "forward kinematics\n"
                << body_pose_fk.translation() << std::endl;
      std::cout << std::endl;
      // This error bound is chosen as tight as possible.

      EXPECT_TRUE(CompareMatrices(body_pose_fk.translation(),
                                  body_pos_global_ik,
                                  pos_tol,
                                  MatrixCompareType::absolute));
      EXPECT_TRUE(CompareMatrices(body_pose_fk.linear(), body_Ri, orient_tol,
                                  MatrixCompareType::absolute));
    }
  }

 protected:
  std::unique_ptr<RigidBodyTree<double>> rigid_body_tree_;
  GlobalInverseKinematics global_ik_;
  int ee_idx_;  // end effector's body index.
};

TEST_F(KukaTest, ReachableTest) {
  // Test the case that global IK should find a solution.
  // "ee" stands for "end effector".
  Eigen::Vector3d ee_pos_lb_W(0.4, -0.1, 0.4);
  Eigen::Vector3d ee_pos_ub_W(0.6, 0.1, 0.6);
  global_ik_.AddWorldPositionConstraint(ee_idx_, Vector3d::Zero(), ee_pos_lb_W,
                                        ee_pos_ub_W);

  Eigen::Quaterniond ee_desired_orient(
      Eigen::AngleAxisd(-M_PI / 2, Vector3d(0, 1, 0)));
  global_ik_.AddWorldOrientationConstraint(ee_idx_, ee_desired_orient,
                                           0.2 * M_PI);

  solvers::GurobiSolver gurobi_solver;
  if (gurobi_solver.available()) {
    global_ik_.SetSolverOption(solvers::SolverType::kGurobi, "OutputFlag", 1);

    SolutionResult sol_result = gurobi_solver.Solve(global_ik_);

    EXPECT_EQ(sol_result, SolutionResult::kSolutionFound);

    double pos_tol = 0.03;
    double orient_tol = 0.1;
    CheckGlobalIKSolution(pos_tol, orient_tol);
  }
}

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
    global_ik_.SetSolverOption(solvers::SolverType::kGurobi, "OutputFlag", 1);

    SolutionResult sol_result = gurobi_solver.Solve(global_ik_);

    EXPECT_TRUE(sol_result == SolutionResult::kInfeasible_Or_Unbounded
                    || sol_result == SolutionResult::kInfeasibleConstraints);
  }
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
    global_ik_.SetSolverOption(solvers::SolverType::kGurobi, "OutputFlag", 1);
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

    // The position tolerance and the orientation tolerance is chosen
    // according to the Gurobi solver tolerance.
    double position_error = 1E-5;
    double orientation_error = 2E-5;
    CheckGlobalIKSolution(position_error, orientation_error);

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
