#include "drake/multibody/optimization/spatial_velocity_constraint.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/inverse_kinematics/test/inverse_kinematics_test_utilities.h"

namespace drake {
namespace multibody {
namespace {

using Eigen::Vector3d;
using Eigen::VectorXd;

TEST_F(IiwaKinematicConstraintTest, SpatialVelocityConstraint) {
  const Vector3d p_BQ(0.1, 0.2, 0.3);
  const Vector3d v_AQ_lower(-0.2, -0.3, -0.4);
  const Vector3d v_AQ_upper(0.2, 0.3, 0.4);
  const auto& frameA = plant_autodiff_->GetFrameByName("iiwa_link_7");
  const auto& frameB = plant_autodiff_->GetFrameByName("iiwa_link_3");

  SpatialVelocityConstraint constraint_translation_only(
      plant_autodiff_.get(), frameA, v_AQ_lower, v_AQ_upper, frameB, p_BQ,
      plant_context_autodiff_.get());

  EXPECT_TRUE(
      CompareMatrices(constraint_translation_only.lower_bound(), v_AQ_lower));
  EXPECT_TRUE(
      CompareMatrices(constraint_translation_only.upper_bound(), v_AQ_upper));

  SpatialVelocityConstraint::AngularVelocityBounds w_AQ_bounds;
  w_AQ_bounds.magnitude_lower = 0.1;
  w_AQ_bounds.magnitude_upper = 1.0;
  w_AQ_bounds.nominal_direction = {0.1, 0.2, 0.3};
  w_AQ_bounds.theta_bound = 0.5;

  SpatialVelocityConstraint constraint(
      plant_autodiff_.get(), frameA, v_AQ_lower, v_AQ_upper, frameB, p_BQ,
      plant_context_autodiff_.get(), w_AQ_bounds);

  VectorXd expected_lower(5), expected_upper(5);
  expected_lower << v_AQ_lower, w_AQ_bounds.magnitude_lower, 0;
  expected_upper << v_AQ_upper, w_AQ_bounds.magnitude_upper,
      w_AQ_bounds.theta_bound;
  EXPECT_TRUE(CompareMatrices(constraint.lower_bound(), expected_lower));
  EXPECT_TRUE(CompareMatrices(constraint.upper_bound(), expected_upper));

  // Set arbitrary positions and velocities.
  VectorXd x(14);
  x << 0.1, 0.2, 0.25, -0.6, 0.26, 0.12, 0.15, 0.36, 0.73, 0.5, -0.62, 0.243,
      0.245, 0.55;

  VectorXd y;
  constraint.Eval(x, &y);

  // Now compute the expected quantities (a different way). We only check that
  // the double values match; the gradients are computed via a straight-forward
  // application of autodiff.
  plant_->SetPositionsAndVelocities(plant_context_, x);
  const Body<double>& bodyA = plant_->GetBodyByName("iiwa_link_7");
  const Body<double>& bodyB = plant_->GetBodyByName("iiwa_link_3");
  const auto V_AW_W = plant_->world_frame().CalcSpatialVelocity(
      *plant_context_, bodyA.body_frame(), plant_->world_frame());
  const auto V_WB_W = bodyB.EvalSpatialVelocityInWorld(*plant_context_);
  const auto R_WB = bodyB.EvalPoseInWorld(*plant_context_).rotation();
  const auto V_WQ_W = V_WB_W.Shift(R_WB * p_BQ);
  const auto R_AW = bodyA.EvalPoseInWorld(*plant_context_).rotation().inverse();
  Vector3d p_WQ_W;
  plant_->CalcPointsPositions(*plant_context_, bodyB.body_frame(), p_BQ,
                              plant_->world_frame(), &p_WQ_W);
  const auto V_AQ_W = V_AW_W.ComposeWithMovingFrameVelocity(p_WQ_W, V_WQ_W);
  const auto V_AQ_A = R_AW * V_AQ_W;

  log()->info("V_WB_W = {}", V_WB_W);
  log()->info("V_WQ_W = {}", V_WQ_W);
  log()->info("p_WQ_W = {}", p_WQ_W.transpose());
  log()->info("p_WQ_A = {}", (R_AW * p_WQ_W).transpose());
  log()->info("V_AW_W = {}", V_AW_W);
  log()->info("V_AQ_W = {}", V_AQ_W);
  log()->info("V_AQ_A = {}", V_AQ_A);

  EXPECT_TRUE(CompareMatrices(V_AQ_A.translational(), y.head<3>(), 1e-14));
  EXPECT_NEAR(y[3], V_AQ_A.rotational().norm(), 1e-14);
  EXPECT_NEAR(std::cos(y[4]),
              V_AQ_A.rotational().normalized().dot(
                  w_AQ_bounds.nominal_direction.normalized()),
              1e-14);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
