#include "drake/multibody/inverse_kinematics/position_constraint.h"

#include <gtest/gtest.h>

#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

using drake::systems::Context;

namespace drake {
namespace multibody {
namespace {
AutoDiffVecXd EvalPositionConstraintAutoDiff(
    const Context<AutoDiffXd>& context, const MultibodyPlant<AutoDiffXd>& plant,
    const Frame<AutoDiffXd>& frameA, const Frame<AutoDiffXd>& frameB,
    const Vector3<double>& p_BQ) {
  Vector3<AutoDiffXd> y_autodiff;
  plant.CalcPointsPositions(context, frameB, p_BQ.cast<AutoDiffXd>(), frameA,
                            &y_autodiff);
  return y_autodiff;
}

TEST_F(IiwaKinematicConstraintTest, PositionConstraint) {
  const Eigen::Vector3d p_BQ(0.1, 0.2, 0.3);
  const Eigen::Vector3d p_AQ_lower(-0.2, -0.3, -0.4);
  const Eigen::Vector3d p_AQ_upper(0.2, 0.3, 0.4);
  const auto frameA_index = plant_->GetFrameByName("iiwa_link_7").index();
  const auto frameB_index = plant_->GetFrameByName("iiwa_link_3").index();
  const Frame<double>& frameA = plant_->get_frame(frameA_index);
  const Frame<double>& frameB = plant_->get_frame(frameB_index);
  PositionConstraint constraint(plant_, frameA, p_AQ_lower, p_AQ_upper, frameB,
                                p_BQ, plant_context_);

  EXPECT_EQ(constraint.num_vars(), plant_->num_positions());
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
  plant_->CalcPointsPositions(*plant_context_, frameB, p_BQ, frameA,
                              &y_expected);
  const double tol = 1E-12;
  EXPECT_TRUE(CompareMatrices(y, y_expected, tol));

  VectorX<AutoDiffXd> q_autodiff = math::InitializeAutoDiff(q);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(q_autodiff, &y_autodiff);
  plant_autodiff_->GetMutablePositions(plant_context_autodiff_.get()) =
      q_autodiff;
  Vector3<AutoDiffXd> y_autodiff_expected = EvalPositionConstraintAutoDiff(
      *plant_context_autodiff_, *plant_autodiff_,
      plant_autodiff_->GetFrameByName(frameA.name()),
      plant_autodiff_->GetFrameByName(frameB.name()), p_BQ);
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, tol);

  // Test with non-identity gradient for q_autodiff.
  q_autodiff = math::InitializeAutoDiff(q, MatrixX<double>::Ones(q.size(), 2));
  plant_autodiff_->GetMutablePositions(plant_context_autodiff_.get()) =
      q_autodiff;
  constraint.Eval(q_autodiff, &y_autodiff);
  y_autodiff_expected = EvalPositionConstraintAutoDiff(
      *plant_context_autodiff_, *plant_autodiff_,
      plant_autodiff_->GetFrameByName(frameA.name()),
      plant_autodiff_->GetFrameByName(frameB.name()), p_BQ);
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, 1E-12);

  // Checks if the constraint constructed from MBP<ADS> gives the same result
  // as from MBP<double>.
  PositionConstraint constraint_from_autodiff(
      plant_autodiff_.get(), plant_autodiff_->get_frame(frameA_index),
      p_AQ_lower, p_AQ_upper, plant_autodiff_->get_frame(frameB_index), p_BQ,
      plant_context_autodiff_.get());
  // Set dq to arbitrary value.
  Eigen::Matrix<double, 7, 2> dq;
  for (int i = 0; i < 7; ++i) {
    dq(i, 0) = i * 2 + 1;
    dq(i, 1) = std::sin(i + 0.2);
  }
  /* tolerance for checking numerical gradient vs analytical gradient. The
   * numerical gradient is only accurate up to 4E-7 */
  // N.B. Using libsdformat9 with pose frame semantics requires an increase in
  // tolerance from 2E-7 to 4E-7.
  const double gradient_tol = 4E-7;
  TestKinematicConstraintEval(constraint, constraint_from_autodiff, q, dq,
                              gradient_tol);
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
  plant_->GetMutablePositions(plant_context_) = q;
  const Eigen::Vector3d p_BQ(0.2, 0.3, 0.4);
  Eigen::Vector3d p_AQ;
  plant_->CalcPointsPositions(
      *plant_context_, plant_->get_frame(body1_index_), p_BQ,
      plant_->get_frame(body2_index_), &p_AQ);

  {
    PositionConstraint good_constraint(
        plant_, plant_->get_frame(body2_index_),
        p_AQ - Eigen::Vector3d::Constant(0.001),
        p_AQ + Eigen::Vector3d::Constant(0.001),
        plant_->get_frame(body1_index_), p_BQ, plant_context_);
    EXPECT_TRUE(good_constraint.CheckSatisfied(q));
  }
  {
    PositionConstraint bad_constraint(
        plant_, plant_->get_frame(body2_index_),
        p_AQ - Eigen::Vector3d::Constant(0.002),
        p_AQ - Eigen::Vector3d::Constant(0.001),
        plant_->get_frame(body1_index_), p_BQ, plant_context_);
    EXPECT_FALSE(bad_constraint.CheckSatisfied(q));
  }
}
}  // namespace
}  // namespace multibody
}  // namespace drake
