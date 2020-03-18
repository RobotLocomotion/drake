#include "drake/multibody/inverse_kinematics/point_to_point_distance_constraint.h"

#include <gtest/gtest.h>

#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

using drake::systems::Context;

namespace drake {
namespace multibody {
namespace {
TEST_F(IiwaKinematicConstraintTest, PointToPointDistanceConstraint) {
  const Eigen::Vector3d p_B1P1(0.1, 0.2, 0.3);
  const Eigen::Vector3d p_B2P2(0.2, -0.3, 0.4);
  const auto frame1_index = plant_->GetFrameByName("iiwa_link_7").index();
  const auto frame2_index = plant_->GetFrameByName("iiwa_link_3").index();
  const Frame<double>& frame1 = plant_->get_frame(frame1_index);
  const Frame<double>& frame2 = plant_->get_frame(frame2_index);
  const double distance_lower = 0.1;
  const double distance_upper = 0.2;
  PointToPointDistanceConstraint constraint(plant_, frame1, p_B1P1, frame2,
                                            p_B2P2, distance_lower,
                                            distance_upper, plant_context_);
  EXPECT_EQ(constraint.num_vars(), plant_->num_positions());
  EXPECT_EQ(constraint.num_constraints(), 1);
  EXPECT_EQ(constraint.lower_bound()(0), distance_lower * distance_lower);
  EXPECT_EQ(constraint.upper_bound()(0), distance_upper * distance_upper);

  // Now check if Eval function computes the right result.
  Eigen::VectorXd q(7);
  q << 0.1, 0.2, 0.3, 0.4, -0.1, -0.2, -0.4;
  Eigen::VectorXd y;
  constraint.Eval(q, &y);

  Eigen::Vector3d p_WP1, p_WP2;
  plant_->SetPositions(plant_context_, q);
  plant_->CalcPointsPositions(*plant_context_, frame1, p_B1P1,
                              plant_->world_frame(), &p_WP1);
  plant_->CalcPointsPositions(*plant_context_, frame2, p_B2P2,
                              plant_->world_frame(), &p_WP2);
  EXPECT_NEAR(y(0), (p_WP1 - p_WP2).squaredNorm(), 1e-14);

  Eigen::MatrixXd dqdz(7, 2);
  dqdz << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 1.1, 1.2, 1.3, 1.4;
  VectorX<AutoDiffXd> q_autodiff =
      math::initializeAutoDiffGivenGradientMatrix(q, dqdz);
  AutoDiffVecXd y_autodiff;
  constraint.Eval(q_autodiff, &y_autodiff);
  plant_autodiff_->SetPositions(plant_context_autodiff_.get(), q_autodiff);
  Vector3<AutoDiffXd> p_B1P2_autodiff;
  plant_autodiff_->CalcPointsPositions(
      *plant_context_autodiff_, plant_autodiff_->get_frame(frame2_index),
      p_B2P2.cast<AutoDiffXd>(), plant_autodiff_->get_frame(frame1_index),
      &p_B1P2_autodiff);
  Vector1<AutoDiffXd> y_autodiff_expected;
  y_autodiff_expected(0) = (p_B1P2_autodiff - p_B1P1).squaredNorm();
  CompareAutoDiffVectors(y_autodiff, y_autodiff_expected, 1e-12);
}

TEST_F(TwoFreeBodiesConstraintTest, PointToPointDistanceConstraint) {
  // Given two free bodies with some given (arbitrary) poses, check if the poses
  // satisfy given point to point distance constraint.
  const Eigen::Quaterniond body1_quaternion(Eigen::AngleAxisd(
      0.3 * M_PI, Eigen::Vector3d(0.1, 0.3, 0.2).normalized()));
  const Eigen::Quaterniond body2_quaternion(Eigen::AngleAxisd(
      -0.2 * M_PI, Eigen::Vector3d(0.4, 1.5, -0.2).normalized()));
  const Eigen::Vector3d body1_position(0.4, -0.02, 3.5);
  const Eigen::Vector3d body2_position(-.1, -2.3, 0.05);
  Eigen::Matrix<double, 14, 1> q;
  q << QuaternionToVectorWxyz(body1_quaternion), body1_position,
      QuaternionToVectorWxyz(body2_quaternion), body2_position;
  plant_->SetPositions(plant_context_, q);
  const Eigen::Vector3d p_B1P1(0.2, 0.3, 0.4);
  const Eigen::Vector3d p_B2P2(-0.1, -0.2, -0.3);
  Eigen::Vector3d p_WP1, p_WP2;
  plant_->CalcPointsPositions(*plant_context_, plant_->get_frame(body1_index_),
                              p_B1P1, plant_->world_frame(), &p_WP1);
  plant_->CalcPointsPositions(*plant_context_, plant_->get_frame(body2_index_),
                              p_B2P2, plant_->world_frame(), &p_WP2);
  const double distance = (p_WP1 - p_WP2).norm();
  PointToPointDistanceConstraint good_constraint(
      plant_, plant_->get_frame(body1_index_), p_B1P1,
      plant_->get_frame(body2_index_), p_B2P2, 0.5 * distance, 2 * distance,
      plant_context_);
  EXPECT_TRUE(good_constraint.CheckSatisfied(q));
  PointToPointDistanceConstraint bad_constraint1(
      plant_, plant_->get_frame(body1_index_), p_B1P1,
      plant_->get_frame(body2_index_), p_B2P2, 1.01 * distance, 2 * distance,
      plant_context_);
  EXPECT_FALSE(bad_constraint1.CheckSatisfied(q));
  PointToPointDistanceConstraint bad_constraint2(
      plant_, plant_->get_frame(body1_index_), p_B1P1,
      plant_->get_frame(body2_index_), p_B2P2, 0.5 * distance, 0.99 * distance,
      plant_context_);
  EXPECT_FALSE(bad_constraint2.CheckSatisfied(q));
}
}  // namespace
}  // namespace multibody
}  // namespace drake
