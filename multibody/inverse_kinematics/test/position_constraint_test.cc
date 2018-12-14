#include "drake/multibody/inverse_kinematics/position_constraint.h"

#include <gtest/gtest.h>

#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

using drake::multibody::internal::IiwaKinematicConstraintTest;
using drake::multibody::internal::TwoFreeBodiesConstraintTest;
using drake::systems::Context;

namespace drake {
namespace multibody {
namespace {
AutoDiffVecXd EvalPositionConstraintAutoDiff(
    const Context<AutoDiffXd>& context, const MultibodyTree<AutoDiffXd>& tree,
    const Frame<AutoDiffXd>& frameA, const Frame<AutoDiffXd>& frameB,
    const Vector3<double>& p_BQ) {
  Vector3<AutoDiffXd> y_autodiff;
  tree.CalcPointsPositions(context, frameB, p_BQ.cast<AutoDiffXd>(), frameA,
                           &y_autodiff);
  return y_autodiff;
}
TEST_F(IiwaKinematicConstraintTest, PositionConstraint) {
  const Eigen::Vector3d p_BQ(0.1, 0.2, 0.3);
  const Eigen::Vector3d p_AQ_lower(-0.2, -0.3, -0.4);
  const Eigen::Vector3d p_AQ_upper(0.2, 0.3, 0.4);
  const Frame<double>& frameB = plant_->GetFrameByName("iiwa_link_7");
  const Frame<double>& frameA = plant_->GetFrameByName("iiwa_link_3");
  PositionConstraint constraint(plant_, frameA, p_AQ_lower, p_AQ_upper, frameB,
                                p_BQ, plant_context_);

  EXPECT_EQ(constraint.num_vars(), iiwa_autodiff_.tree().num_positions());
  EXPECT_EQ(constraint.num_constraints(), 3);
  EXPECT_EQ(constraint.lower_bound(), p_AQ_lower);
  EXPECT_EQ(constraint.upper_bound(), p_AQ_upper);

  // Now check if Eval function computes the right result.
  Eigen::VectorXd q(7);
  q << 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.3;
  Eigen::VectorXd y;
  constraint.Eval(q, &y);

  Eigen::MatrixXd y_expected(3, 1);

  plant_->SetPositions(plant_context_, q);
  plant_->tree().CalcPointsPositions(*plant_context_, frameB, p_BQ, frameA,
                                     &y_expected);
  const double tol = 1E-12;
  EXPECT_TRUE(CompareMatrices(y, y_expected, tol));

  VectorX<AutoDiffXd> q_autodiff = math::initializeAutoDiff(q);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(q_autodiff, &y_autodiff);
  auto mbt_context_autodiff =
      dynamic_cast<MultibodyTreeContext<AutoDiffXd>*>(context_autodiff_.get());
  mbt_context_autodiff->get_mutable_positions() = q_autodiff;
  Vector3<AutoDiffXd> y_autodiff_expected = EvalPositionConstraintAutoDiff(
      *context_autodiff_, iiwa_autodiff_.tree(),
      iiwa_autodiff_.tree().GetFrameByName(frameA.name()),
      iiwa_autodiff_.tree().GetFrameByName(frameB.name()), p_BQ);
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, tol);

  // Test with non-identity gradient for q_autodiff.
  q_autodiff = math::initializeAutoDiffGivenGradientMatrix(
      q, MatrixX<double>::Ones(q.size(), 2));
  mbt_context_autodiff->get_mutable_positions() = q_autodiff;
  constraint.Eval(q_autodiff, &y_autodiff);
  y_autodiff_expected = EvalPositionConstraintAutoDiff(
      *context_autodiff_, iiwa_autodiff_.tree(),
      iiwa_autodiff_.tree().GetFrameByName(frameA.name()),
      iiwa_autodiff_.tree().GetFrameByName(frameB.name()), p_BQ);
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, 1E-12);
}

TEST_F(TwoFreeBodiesConstraintTest, PositionConstraint) {
  // Given two free bodies with some given (arbitrary) poses, check if the poses
  // satisfy given position constraints.
  const Eigen::Quaterniond body1_quaternion(Eigen::AngleAxisd(
      0.3 * M_PI, Eigen::Vector3d(0.1, 0.3, 0.2).normalized()));
  const Eigen::Quaterniond body2_quaternion(Eigen::AngleAxisd(
      -0.2 * M_PI, Eigen::Vector3d(0.4, 1.5, -0.2).normalized()));
  const Eigen::Vector3d body1_position(0.4, -0.02, 3.5);
  const Eigen::Vector3d body2_position(-.1, -2.3, 0.05);
  Eigen::Matrix<double, 14, 1> q;
  q << QuaternionToVectorWxyz(body1_quaternion), body1_position,
      QuaternionToVectorWxyz(body2_quaternion), body2_position;
  dynamic_cast<MultibodyTreeContext<double>*>(context_double_.get())
      ->get_mutable_positions() = q;
  const Eigen::Vector3d p_BQ(0.2, 0.3, 0.4);
  Eigen::Vector3d p_AQ;
  two_bodies_double_.tree().CalcPointsPositions(
      *context_double_, two_bodies_double_.tree().get_frame(body1_index_), p_BQ,
      two_bodies_double_.tree().get_frame(body2_index_), &p_AQ);

  {
    PositionConstraint good_constraint(
        plant_, plant_->tree().get_frame(body2_index_),
        p_AQ - Eigen::Vector3d::Constant(0.001),
        p_AQ + Eigen::Vector3d::Constant(0.001),
        plant_->tree().get_frame(body1_index_), p_BQ, plant_context_);
    EXPECT_TRUE(good_constraint.CheckSatisfied(q));
  }
  {
    PositionConstraint bad_constraint(
        plant_, plant_->tree().get_frame(body2_index_),
        p_AQ - Eigen::Vector3d::Constant(0.002),
        p_AQ - Eigen::Vector3d::Constant(0.001),
        plant_->tree().get_frame(body1_index_), p_BQ, plant_context_);
    EXPECT_FALSE(bad_constraint.CheckSatisfied(q));
  }
}
}  // namespace
}  // namespace multibody
}  // namespace drake
