#include "drake/multibody/inverse_kinematics/test/global_inverse_kinematics_test_util.h"

#include <string>
#include <vector>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/solve.h"
#include "drake/systems/framework/diagram_builder.h"

using Eigen::Isometry3d;
using Eigen::Vector3d;

using drake::geometry::SceneGraph;
using drake::solvers::SolutionResult;

namespace drake {
namespace multibody {
std::unique_ptr<multibody::MultibodyPlant<double>> ConstructKuka() {
  const std::string iiwa_path = FindResourceOrThrow(
      "drake/manipulation/models/iiwa_description/sdf/"
      "iiwa14_no_collision.sdf");
  auto plant = std::make_unique<MultibodyPlant<double>>(0.1);
  multibody::Parser parser{plant.get()};
  parser.AddModelFromFile(iiwa_path, "iiwa");
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("iiwa_link_0"));
  plant->Finalize();

  return plant;
}

std::unique_ptr<MultibodyPlant<double>> ConstructSingleBody() {
  const double mass{1};
  const Eigen::Vector3d p_AoAcm_A(0, 0, 0);
  const RotationalInertia<double> I_AAcm_A{0.001, 0.001, 0.001};
  const SpatialInertia<double> M_AAo_A =
      SpatialInertia<double>::MakeFromCentralInertia(mass, p_AoAcm_A, I_AAcm_A);
  auto plant = std::make_unique<MultibodyPlant<double>>(0.1);
  plant->AddRigidBody("body1", M_AAo_A);
  plant->Finalize();
  return plant;
}

KukaTest::KukaTest()
    : plant_(ConstructKuka()),
      global_ik_(*plant_),  // Test with default options.
                            // half axis.
      ee_idx_(plant_->GetBodyByName("iiwa_link_7").index()) {}

void KukaTest::CheckGlobalIKSolution(
    const solvers::MathematicalProgramResult& result, double pos_tol,
    double orient_tol) const {
  Eigen::VectorXd q_global_ik =
      global_ik_.ReconstructGeneralizedPositionSolution(result);

  auto context = plant_->CreateDefaultContext();
  plant_->SetPositions(context.get(), q_global_ik);

  // TODO(hongkai.dai): replace this print out with error check. We have not
  // derived a rigorous bound on the rotation matrix relaxation yet. Should be
  // able to get a more meaningful bound when we have some theoretical proof.
  for (BodyIndex i{1}; i < plant_->num_bodies(); ++i) {
    // Compute forward kinematics.
    const auto& body_pose_fk = plant_->CalcRelativeTransform(
        *context, plant_->world_frame(),
        plant_->get_body(BodyIndex{i}).body_frame());

    const Eigen::Matrix3d body_Ri =
        result.GetSolution(global_ik_.body_rotation_matrix(i));
    // Tolerance from Gurobi is about 1E-6. I increase it to 3e-6 to pass on Mac
    // CI.
    const double tol = 3e-6;
    EXPECT_TRUE((body_Ri.array().abs() <= 1 + tol).all()) << "body_Ri:\n"
                                                          << body_Ri << "\n";
    EXPECT_LE(body_Ri.trace(), 3 + tol);
    EXPECT_GE(body_Ri.trace(), -1 - tol);
    Vector3d body_pos_global_ik =
        result.GetSolution(global_ik_.body_position(i));
    EXPECT_TRUE(CompareMatrices(body_pose_fk.translation(), body_pos_global_ik,
                                pos_tol, MatrixCompareType::absolute));
    EXPECT_TRUE(CompareMatrices(body_pose_fk.rotation().matrix(), body_Ri,
                                orient_tol, MatrixCompareType::absolute));
  }
}

Eigen::VectorXd KukaTest::CheckNonlinearIK(
    const Eigen::Vector3d& ee_pos_lb_W, const Eigen::Vector3d& ee_pos_ub_W,
    const Eigen::Quaterniond& ee_orient, double angle_tol,
    const Eigen::Matrix<double, 7, 1>& q_guess,
    const Eigen::Matrix<double, 7, 1>& q_nom, bool ik_success_expected) const {
  InverseKinematics ik(*plant_);
  ik.AddPositionConstraint(plant_->get_body(BodyIndex{ee_idx_}).body_frame(),
                           Eigen::Vector3d::Zero(), plant_->world_frame(),
                           ee_pos_lb_W, ee_pos_ub_W);
  ik.AddOrientationConstraint(
      plant_->get_body(BodyIndex{ee_idx_}).body_frame(),
      math::RotationMatrix<double>::Identity(), plant_->world_frame(),
      math::RotationMatrix<double>(ee_orient), angle_tol);
  Eigen::VectorXd q_sol(7);
  Eigen::VectorXd q_guess_ik = q_guess;
  Eigen::VectorXd q_nom_ik = q_nom;
  ik.get_mutable_prog()->AddQuadraticErrorCost(
      Eigen::MatrixXd::Identity(plant_->num_positions(),
                                plant_->num_positions()),
      q_nom_ik, ik.q());
  const auto result = solvers::Solve(ik.prog(), q_guess_ik);
  EXPECT_EQ(result.is_success(), ik_success_expected);
  return q_sol;
}
}  // namespace multibody
}  // namespace drake
