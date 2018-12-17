#include "drake/multibody/inverse_kinematics/orientation_constraint.h"

#include <gtest/gtest.h>

#include "drake/math/rotation_matrix.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

using drake::math::RotationMatrixd;
using drake::multibody::internal::IiwaKinematicConstraintTest;
using drake::multibody::internal::TwoFreeBodiesConstraintTest;
using drake::systems::Context;

namespace drake {
namespace multibody {
namespace {
AutoDiffVecXd EvalOrientationConstraintAutoDiff(
    const Context<AutoDiffXd>& context, const MultibodyTree<AutoDiffXd>& tree,
    const Frame<AutoDiffXd>& frameAbar, const RotationMatrixd& R_AbarA,
    const Frame<AutoDiffXd>& frameBbar, const RotationMatrixd& R_BbarB) {
  Vector1<AutoDiffXd> y_autodiff(1);
  const Matrix3<AutoDiffXd> R_AbarBbar =
      tree.CalcRelativeTransform(context, frameAbar, frameBbar).linear();
  const Matrix3<AutoDiffXd> R_AB =
      R_AbarA.matrix().cast<AutoDiffXd>().transpose() * R_AbarBbar *
      R_BbarB.matrix().cast<AutoDiffXd>();
  y_autodiff(0) = R_AB.trace();
  return y_autodiff;
}

TEST_F(IiwaKinematicConstraintTest, OrientationConstraint) {
  const double angle_bound{0.1 * M_PI};
  const Frame<double>& frameAbar = plant_->GetFrameByName("iiwa_link_7");
  const Frame<double>& frameBbar = plant_->GetFrameByName("iiwa_link_3");
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
  EXPECT_EQ(constraint->num_vars(), iiwa_autodiff_.tree().num_positions());
  EXPECT_TRUE(CompareMatrices(constraint->lower_bound(),
                              Vector1d(2 * std::cos(angle_bound) + 1)));
  EXPECT_TRUE(CompareMatrices(constraint->upper_bound(), Vector1d(3)));

  Eigen::VectorXd q(iiwa_autodiff_.tree().num_positions());
  // Arbitrary joint angles.
  q << 0.1, 0.2, 0.3, 0.4, 0.5, -0.3, -0.2;
  AutoDiffVecXd q_autodiff = math::initializeAutoDiff(q);
  AutoDiffVecXd y_autodiff;
  constraint->Eval(q_autodiff, &y_autodiff);

  auto mbt_context_autodiff =
      dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(context_autodiff_.get());
  mbt_context_autodiff->get_mutable_positions() = q_autodiff;
  AutoDiffVecXd y_autodiff_expected = EvalOrientationConstraintAutoDiff(
      *context_autodiff_, iiwa_autodiff_.tree(),
      iiwa_autodiff_.tree().GetFrameByName(frameAbar.name()), R_AbarA,
      iiwa_autodiff_.tree().GetFrameByName(frameBbar.name()), R_BbarB);
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, 1E-12);

  // Test with non-identity gradient for q_autodiff.
  q_autodiff = math::initializeAutoDiffGivenGradientMatrix(
      q, MatrixX<double>::Ones(q.size(), 2));
  mbt_context_autodiff->get_mutable_positions() = q_autodiff;
  constraint->Eval(q_autodiff, &y_autodiff);
  y_autodiff_expected = EvalOrientationConstraintAutoDiff(
      *context_autodiff_, iiwa_autodiff_.tree(),
      iiwa_autodiff_.tree().GetFrameByName(frameAbar.name()), R_AbarA,
      iiwa_autodiff_.tree().GetFrameByName(frameBbar.name()), R_BbarB);
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, 1E-12);
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
      plant_, plant_->tree().get_frame(body1_index_), R_AbarA,
      plant_->tree().get_frame(body2_index_), R_BbarB, theta * 1.01,
      plant_context_);
  EXPECT_TRUE(good_constraint.CheckSatisfied(q));

  OrientationConstraint bad_constraint(
      plant_, plant_->tree().get_frame(body1_index_), R_AbarA,
      plant_->tree().get_frame(body2_index_), R_BbarB, theta * 0.99,
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
