#include "drake/multibody/global_inverse_kinematics.h"

#include <gtest/gtest.h>
#include <drake/solvers/gurobi_solver.h>

#include "drake/common/drake_path.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree_construction.h"

using Eigen::Vector3d;
using Eigen::Isometry3d;

using drake::solvers::SolutionResult;

namespace drake {
namespace multibody {
namespace {
GTEST_TEST(TestGlobakIK, KukaTest) {
  auto rigid_body_tree = std::make_unique<RigidBodyTree<double>>();
  const std::string model_path = drake::GetDrakePath() +
      "/examples/kuka_iiwa_arm/models/iiwa14/"
          "iiwa14_simplified_collision.urdf";

  parsers::urdf::AddModelInstanceFromUrdfFile(
    model_path,
    drake::multibody::joints::kFixed,
    nullptr,
    rigid_body_tree.get());

  AddFlatTerrainToWorld(rigid_body_tree.get());

  GlobalInverseKinematics global_ik(*rigid_body_tree, 2);

  int ee_idx = rigid_body_tree->FindBodyIndex("iiwa_link_ee");
  Eigen::Vector3d ee_pos_lb(0.4, -0.1, 0.4);
  Eigen::Vector3d ee_pos_ub(0.6, 0.1, 0.6);
  global_ik.AddWorldPositionConstraint(ee_idx, Vector3d::Zero(), ee_pos_lb, ee_pos_ub);

  Eigen::Quaterniond ee_desired_orient(Eigen::AngleAxisd(-M_PI / 2, Vector3d(0, 1, 0)));
  global_ik.AddWorldOrientationConstraint(ee_idx, ee_desired_orient, 0.2 * M_PI);

  solvers::GurobiSolver gurobi_solver;

  global_ik.SetSolverOption(solvers::SolverType::kGurobi, "OutputFlag", 1);
  SolutionResult sol_result = gurobi_solver.Solve(global_ik);

  EXPECT_EQ(sol_result, SolutionResult::kSolutionFound);

  const auto& body_pos = global_ik.body_pos();

  const auto& body_rotmat = global_ik.body_rotmat();

  Eigen::VectorXd q_global_ik = global_ik.ReconstructPostureSolution();

  std::cout<<"Reconstructed robot posture:\n" << q_global_ik << std::endl;
  KinematicsCache<double> cache = rigid_body_tree->doKinematics(q_global_ik);

  for (int i = 1; i < rigid_body_tree->get_num_bodies(); ++i) {
    const auto& body_pose_fk = rigid_body_tree->CalcFramePoseInWorldFrame(cache, rigid_body_tree->get_body(i), Isometry3d::Identity());

    const Eigen::Matrix3d body_Ri = global_ik.GetSolution((body_rotmat[i]));
    std::cout << rigid_body_tree->get_body(i).get_name() << std::endl;
    std::cout << "rotation matrix:\n global_ik\n" << body_Ri << std::endl;
    std::cout << "forward kinematics\n" << body_pose_fk.linear() << std::endl;
    std::cout << "R * R':\n" << body_Ri * body_Ri.transpose() << std::endl;
    std::cout << "det(R) = " << body_Ri.determinant() << std::endl;
    std::cout << "position:\n global_ik\n" << global_ik.GetSolution(body_pos[i]) << std::endl;
    std::cout << "forward kinematics\n" << body_pose_fk.translation() << std::endl;
    std::cout << std::endl;
  }
}
}  // namespace
}  // namespace multibody
}  // namespace drake