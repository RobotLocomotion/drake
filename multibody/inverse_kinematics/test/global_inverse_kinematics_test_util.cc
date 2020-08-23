#include "drake/multibody/dev/test/global_inverse_kinematics_test_util.h"

#include <string>
#include <vector>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_constraint.h"
#include "drake/multibody/rigid_body_ik.h"
#include "drake/multibody/rigid_body_tree_construction.h"

using Eigen::Vector3d;
using Eigen::Isometry3d;

using drake::solvers::SolutionResult;

namespace drake {
namespace multibody {
std::unique_ptr<RigidBodyTree<double>> ConstructKuka() {
  std::unique_ptr<RigidBodyTree<double>> rigid_body_tree =
      std::make_unique<RigidBodyTree<double>>();
  const std::string model_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/urdf/"
      "iiwa14_polytope_collision.urdf");

  parsers::urdf::AddModelInstanceFromUrdfFile(
      model_path,
      drake::multibody::joints::kFixed,
      nullptr,
      rigid_body_tree.get());

  AddFlatTerrainToWorld(rigid_body_tree.get());
  return rigid_body_tree;
}

std::unique_ptr<RigidBodyTree<double>> ConstructSingleBody() {
  std::unique_ptr<RigidBodyTree<double>> single_object =
      std::make_unique<RigidBodyTree<double>>();
  const std::string model_path =
      FindResourceOrThrow("drake/multibody/models/box.urdf");
  parsers::urdf::AddModelInstanceFromUrdfFile(
      model_path, drake::multibody::joints::kQuaternion, nullptr,
      single_object.get());

  return single_object;
}

KukaTest::KukaTest()
    : rigid_body_tree_(ConstructKuka()),
      global_ik_(*rigid_body_tree_),  // Test with default options.
    // half axis.
      ee_idx_(rigid_body_tree_->FindBodyIndex("iiwa_link_ee")) {}

void KukaTest::CheckGlobalIKSolution(
    const solvers::MathematicalProgramResult& result, double pos_tol,
    double orient_tol) const {
  Eigen::VectorXd q_global_ik =
      global_ik_.ReconstructGeneralizedPositionSolution(result);

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
        result.GetSolution(global_ik_.body_rotation_matrix(i));
    // Tolerance from Gurobi is about 1E-6.
    const double tol = 1e-6;
    EXPECT_TRUE((body_Ri.array().abs() <= 1 + tol).all());
    EXPECT_LE(body_Ri.trace(), 3 + tol);
    EXPECT_GE(body_Ri.trace(), -1 - tol);
    // TODO(hongkai.dai): We will have a more meaningful bound on the
    // relaxation of rotation matrix. Then clean up this print out with
    // the check on the error bound, and move this file out of dev folder.
    std::cout << rigid_body_tree_->get_body(i).get_name() << std::endl;
    std::cout << "rotation matrix:\n global_ik\n" << body_Ri << std::endl;
    std::cout << "forward kinematics\n" << body_pose_fk.linear() << std::endl;
    std::cout << "R * R':\n" << body_Ri * body_Ri.transpose() << std::endl;
    std::cout << "det(R) = " << body_Ri.determinant() << std::endl;
    Vector3d body_pos_global_ik =
        result.GetSolution(global_ik_.body_position(i));
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

Eigen::VectorXd KukaTest::CheckNonlinearIK(
    const Eigen::Vector3d& ee_pos_lb_W, const Eigen::Vector3d& ee_pos_ub_W,
    const Eigen::Quaterniond& ee_orient, double angle_tol,
    const Eigen::Matrix<double, 7, 1>& q_guess,
    const Eigen::Matrix<double, 7, 1>& q_nom, int info_expected) const {
  WorldPositionConstraint pos_cnstr(rigid_body_tree_.get(), ee_idx_,
                                    Eigen::Vector3d::Zero(), ee_pos_lb_W,
                                    ee_pos_ub_W);
  WorldQuatConstraint orient_cnstr(
      rigid_body_tree_.get(), ee_idx_,
      Eigen::Vector4d(ee_orient.w(), ee_orient.x(), ee_orient.y(),
                      ee_orient.z()),
      angle_tol);
  Eigen::VectorXd q_sol(7);
  Eigen::VectorXd q_guess_ik = q_guess;
  Eigen::VectorXd q_nom_ik = q_nom;
  int info;
  IKoptions ikoptions(rigid_body_tree_.get());
  std::vector<std::string> infeasible_constraint;
  std::array<RigidBodyConstraint*, 2> cnstr = {{&pos_cnstr, &orient_cnstr}};
  inverseKin(rigid_body_tree_.get(), q_guess_ik, q_nom_ik, 2, cnstr.data(),
             ikoptions, &q_sol, &info, &infeasible_constraint);
  EXPECT_EQ(info, info_expected);
  return q_sol;
}
}  // namespace multibody
}  // namespace drake
