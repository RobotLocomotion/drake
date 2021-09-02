#include "drake/multibody/inverse_kinematics/orientation_constraint.h"

#include <gtest/gtest.h>

#include "drake/math/rotation_matrix.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

using drake::math::RotationMatrixd;
using drake::systems::Context;

namespace drake {
namespace multibody {
namespace {
AutoDiffVecXd EvalOrientationConstraintAutoDiff(
    const Context<AutoDiffXd>& context, const MultibodyPlant<AutoDiffXd>& plant,
    const Frame<AutoDiffXd>& frameAbar, const RotationMatrixd& R_AbarA,
    const Frame<AutoDiffXd>& frameBbar, const RotationMatrixd& R_BbarB) {
  Vector1<AutoDiffXd> y_autodiff(1);
  const math::RotationMatrix<AutoDiffXd> R_AbarBbar =
      plant.CalcRelativeRotationMatrix(context, frameAbar, frameBbar);
  const math::RotationMatrix<AutoDiffXd> R_AB =
      R_AbarA.cast<AutoDiffXd>().transpose() * R_AbarBbar *
      R_BbarB.cast<AutoDiffXd>();
  y_autodiff(0) = R_AB.matrix().trace();
  return y_autodiff;
}

TEST_F(IiwaKinematicConstraintTest, OrientationConstraint) {
  const double angle_bound{0.1 * M_PI};
  const auto frameAbar_index = plant_->GetFrameByName("iiwa_link_7").index();
  const auto frameBbar_index = plant_->GetFrameByName("iiwa_link_3").index();
  const Frame<double>& frameAbar = plant_->get_frame(frameAbar_index);
  const Frame<double>& frameBbar = plant_->get_frame(frameBbar_index);
  const math::RotationMatrix<double> R_AbarA(Eigen::AngleAxisd(
      0.2 * M_PI, Eigen::Vector3d(0.2, 0.4, -0.5).normalized()));
  const math::RotationMatrix<double> R_BbarB(Eigen::AngleAxisd(
      -0.4 * M_PI, Eigen::Vector3d(0.1, 1.2, -0.7).normalized()));
  std::unique_ptr<OrientationConstraint> constraint{};
  {
    // Construct using local copies of R_AbarA and R_BbarB to ensure that the
    // constraint doesn't rely on the lifetimes of those arguments.
    const math::RotationMatrix<double> R_AbarA_local{R_AbarA};
    const math::RotationMatrix<double> R_BbarB_local{R_BbarB};
    constraint = std::make_unique<OrientationConstraint>(
        plant_, frameAbar, R_AbarA_local, frameBbar, R_BbarB_local, angle_bound,
        plant_context_);
  }

  EXPECT_EQ(constraint->num_constraints(), 1);
  EXPECT_EQ(constraint->num_vars(), plant_autodiff_->num_positions());
  EXPECT_TRUE(CompareMatrices(constraint->lower_bound(),
                              Vector1d(2 * std::cos(angle_bound) + 1)));
  EXPECT_TRUE(CompareMatrices(constraint->upper_bound(), Vector1d(3)));

  Eigen::VectorXd q(plant_autodiff_->num_positions());
  // Arbitrary joint angles.
  q << 0.1, 0.2, 0.3, 0.4, 0.5, -0.3, -0.2;
  AutoDiffVecXd q_autodiff = math::InitializeAutoDiff(q);
  AutoDiffVecXd y_autodiff;
  constraint->Eval(q_autodiff, &y_autodiff);

  plant_autodiff_->GetMutablePositions(plant_context_autodiff_.get()) =
      q_autodiff;
  AutoDiffVecXd y_autodiff_expected = EvalOrientationConstraintAutoDiff(
      *plant_context_autodiff_, *plant_autodiff_,
      plant_autodiff_->GetFrameByName(frameAbar.name()), R_AbarA,
      plant_autodiff_->GetFrameByName(frameBbar.name()), R_BbarB);
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, 1E-12);

  // Test with non-identity gradient for q_autodiff.
  q_autodiff = math::InitializeAutoDiff(q, MatrixX<double>::Ones(q.size(), 2));
  plant_autodiff_->GetMutablePositions(plant_context_autodiff_.get()) =
      q_autodiff;
  constraint->Eval(q_autodiff, &y_autodiff);
  y_autodiff_expected = EvalOrientationConstraintAutoDiff(
      *plant_context_autodiff_, *plant_autodiff_,
      plant_autodiff_->GetFrameByName(frameAbar.name()), R_AbarA,
      plant_autodiff_->GetFrameByName(frameBbar.name()), R_BbarB);
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, 1E-12);

  // Checks if the constraint constructed from MBP<ADS> gives the same result
  // as from MBP<double>.
  const OrientationConstraint constraint_from_autodiff(
      plant_autodiff_.get(), plant_autodiff_->get_frame(frameAbar_index),
      R_AbarA, plant_autodiff_->get_frame(frameBbar_index), R_BbarB,
      angle_bound, plant_context_autodiff_.get());
  // Set dq to arbitrary value.
  Eigen::Matrix<double, 7, 2> dq;
  for (int i = 0; i < 7; ++i) {
    dq(i, 0) = i * 2 + 1;
    dq(i, 1) = std::sin(i + 0.2);
  }
  // tolerance for checking numerical gradient vs analytical gradient.
  // The numerical gradient is only accurate up to 2E-6.
  const double gradient_tol = 2E-6;
  TestKinematicConstraintEval(*constraint, constraint_from_autodiff, q, dq,
                              gradient_tol);
}

TEST_F(TwoFreeBodiesConstraintTest, OrientationConstraint) {
  // Check if the orientation constraints are satisfied for two bodies with
  // given orientations
  const Eigen::Quaterniond body1_quaternion(Eigen::AngleAxisd(
      0.3 * M_PI, Eigen::Vector3d(0.1, 0.3, 0.2).normalized()));
  const Eigen::Quaterniond body2_quaternion(Eigen::AngleAxisd(
      -0.2 * M_PI, Eigen::Vector3d(0.4, 1.5, -0.2).normalized()));
  const Eigen::Vector3d body1_position(0.4, -0.02, 3.5);
  const Eigen::Vector3d body2_position(-.1, -2.3, 0.05);
  Eigen::Matrix<double, 14, 1> q;
  q << QuaternionToVectorWxyz(body1_quaternion), body1_position,
      QuaternionToVectorWxyz(body2_quaternion), body2_position;
  const math::RotationMatrix<double> R_AbarA(Eigen::AngleAxisd(
      0.2 * M_PI, Eigen::Vector3d(0.2, 0.4, -0.5).normalized()));
  const math::RotationMatrix<double> R_BbarB(Eigen::AngleAxisd(
      -0.4 * M_PI, Eigen::Vector3d(0.1, 1.2, -0.7).normalized()));
  const math::RotationMatrixd R_AbarBbar(
      body1_quaternion.inverse() * body2_quaternion);
  const math::RotationMatrixd R_AB = R_AbarA.transpose() * R_AbarBbar * R_BbarB;
  const double theta = R_AB.ToAngleAxis().angle();

  OrientationConstraint good_constraint(
      plant_, plant_->get_frame(body1_index_), R_AbarA,
      plant_->get_frame(body2_index_), R_BbarB, theta * 1.01,
      plant_context_);
  EXPECT_TRUE(good_constraint.CheckSatisfied(q));

  OrientationConstraint bad_constraint(
      plant_, plant_->get_frame(body1_index_), R_AbarA,
      plant_->get_frame(body2_index_), R_BbarB, theta * 0.99,
      plant_context_);
  EXPECT_FALSE(bad_constraint.CheckSatisfied(q));
}
TEST_F(IiwaKinematicConstraintTest, OrientationConstraintConstructionError) {
  // Throws a logic error for negative angle bound.
  EXPECT_THROW(
      OrientationConstraint(plant_, plant_->GetFrameByName("iiwa_link_7"),
                            math::RotationMatrix<double>::Identity(),
                            plant_->GetFrameByName("iiwa_link_3"),
                            math::RotationMatrix<double>::Identity(), -0.01,
                            plant_context_),
      std::invalid_argument);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
