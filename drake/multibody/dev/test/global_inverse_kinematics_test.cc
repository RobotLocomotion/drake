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
      : rigid_body_tree_(ConstructKuka()), global_ik_(*rigid_body_tree_, 2) {}

 protected:
  std::unique_ptr<RigidBodyTree<double>> rigid_body_tree_;
  GlobalInverseKinematics global_ik_;
};

TEST_F(KukaTest, ReachableTest) {
  // Test the case that global IK should find a solution.
  // "ee" stands for "end effector".
  int ee_idx = rigid_body_tree_->FindBodyIndex("iiwa_link_ee");
  // Deliberately uses a large box.
  Eigen::Vector3d ee_pos_lb_W(0.4, -0.1, 0.4);
  Eigen::Vector3d ee_pos_ub_W(0.6, 0.1, 0.6);
  global_ik_.AddWorldPositionConstraint(ee_idx, Vector3d::Zero(), ee_pos_lb_W,
                                        ee_pos_ub_W);

  Eigen::Quaterniond ee_desired_orient(
      Eigen::AngleAxisd(-M_PI / 2, Vector3d(0, 1, 0)));
  // Deliberately uses a large bound.
  global_ik_.AddWorldOrientationConstraint(ee_idx, ee_desired_orient,
                                           0.2 * M_PI);

  solvers::GurobiSolver gurobi_solver;
  if (gurobi_solver.available()) {
    global_ik_.SetSolverOption(solvers::SolverType::kGurobi, "OutputFlag", 1);

    SolutionResult sol_result = gurobi_solver.Solve(global_ik_);

    EXPECT_EQ(sol_result, SolutionResult::kSolutionFound);

    Eigen::VectorXd q_global_ik = global_ik_.ReconstructPostureSolution();

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
      double pos_tol = 0.03;
      double orient_tol = 0.1;
      EXPECT_TRUE(CompareMatrices(body_pose_fk.translation(),
                                  body_pos_global_ik,
                                  pos_tol,
                                  MatrixCompareType::absolute));
      EXPECT_TRUE(CompareMatrices(body_pose_fk.linear(), body_Ri, orient_tol,
                                  MatrixCompareType::absolute));
    }
  }
}

TEST_F(KukaTest, UnreachableTest) {
  // Test a cartesian pose that we know is not reachable.
  int ee_idx = rigid_body_tree_->FindBodyIndex("iiwa_link_ee");
  Eigen::Vector3d ee_pos_lb(0.6, -0.1, 0.7);
  Eigen::Vector3d ee_pos_ub(0.6, 0.1, 0.7);
  global_ik_.AddWorldPositionConstraint(ee_idx, Vector3d::Zero(), ee_pos_lb,
                                        ee_pos_ub);

  Eigen::Quaterniond ee_desired_orient(
      Eigen::AngleAxisd(-M_PI, Vector3d(0, 1, 0)));
  global_ik_.AddWorldOrientationConstraint(ee_idx, ee_desired_orient,
                                           0.0 * M_PI);

  solvers::GurobiSolver gurobi_solver;

  if (gurobi_solver.available()) {
    global_ik_.SetSolverOption(solvers::SolverType::kGurobi, "OutputFlag", 1);

    SolutionResult sol_result = gurobi_solver.Solve(global_ik_);

    EXPECT_TRUE(sol_result == SolutionResult::kInfeasible_Or_Unbounded
                    || sol_result == SolutionResult::kInfeasibleConstraints);
  }
}
}  // namespace
}  // namespace multibody
}  // namespace drake
