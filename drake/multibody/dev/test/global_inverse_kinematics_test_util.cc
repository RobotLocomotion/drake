#include "drake/multibody/dev/test/global_inverse_kinematics_test_util.h"

#include <string>

#include "drake/common/drake_path.h"
#include "drake/common/eigen_matrix_compare.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree_construction.h"

using Eigen::Vector3d;
using Eigen::Isometry3d;

using drake::solvers::SolutionResult;

namespace drake {
namespace multibody {
std::unique_ptr<RigidBodyTree<double>> ConstructKuka() {
  std::unique_ptr<RigidBodyTree<double>> rigid_body_tree =
      std::make_unique<RigidBodyTree<double>>();
  const std::string model_path = drake::GetDrakePath() +
      "/manipulation/models/iiwa_description/urdf/"
          "iiwa14_polytope_collision.urdf";

  parsers::urdf::AddModelInstanceFromUrdfFile(
      model_path,
      drake::multibody::joints::kFixed,
      nullptr,
      rigid_body_tree.get());

  AddFlatTerrainToWorld(rigid_body_tree.get());
  return rigid_body_tree;
}

KukaTest::KukaTest()
    : rigid_body_tree_(ConstructKuka()),
      global_ik_(*rigid_body_tree_, 2),  // Test with 2 binary variables per
    // half axis.
      ee_idx_(rigid_body_tree_->FindBodyIndex("iiwa_link_ee")) {}

void KukaTest::CheckGlobalIKSolution(double pos_tol, double orient_tol) const {
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
    // Use 1E-10 for the error tolerance.
    EXPECT_TRUE((body_Ri.array().abs() <= 1 + 1E-10).all());
    EXPECT_LE(body_Ri.trace(), 3 + 1E-10);
    EXPECT_GE(body_Ri.trace(), -1 - 1E-10);
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
    EXPECT_TRUE(CompareMatrices(body_pose_fk.translation(),
                                body_pos_global_ik,
                                pos_tol,
                                MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(body_pose_fk.linear(), body_Ri, orient_tol,
                                MatrixCompareType::absolute));
  }
}
}  // namespace multibody
}  // namespace drake
