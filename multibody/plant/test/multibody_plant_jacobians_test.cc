#include <limits>
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/test/kuka_iiwa_model_tests.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {

namespace multibody {

using test::KukaIiwaModelTests;

namespace {

TEST_F(KukaIiwaModelTests, FixtureInvariants) {
  // Sanity check basic invariants.
  // Seven dofs for the arm plus floating base.
  EXPECT_EQ(plant_->num_joints(), kNumJoints);
  EXPECT_EQ(plant_->num_positions(), kNumPositions);
  EXPECT_EQ(plant_->num_velocities(), kNumVelocities);
}

// TODO(amcastro-tri): Rename this test as per issue #10155.
TEST_F(KukaIiwaModelTests, CalcPointsAnalyticalJacobianExpressedInWorld) {
  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  SetArbitraryConfiguration();

  // A set of points Pi fixed in the end effector frame E.
  const int kNumPoints = 2;  // The set stores 2 points.
  MatrixX<double> p_EPi(3, kNumPoints);
  p_EPi.col(0) << 0.1, -0.05, 0.02;
  p_EPi.col(1) << 0.2, 0.3, -0.15;

  MatrixX<double> p_WPi(3, kNumPoints);
  MatrixX<double> Jq_WPi(3 * kNumPoints, plant_->num_positions());

  CalcPointsOnEndEffectorAnalyticJacobian(
      *plant_, *context_, p_EPi, &p_WPi, &Jq_WPi);

  // Alternatively, compute the analytic Jacobian by taking the gradient of
  // the positions p_WPi(q) with respect to the generalized positions. We do
  // that with the steps below.

  // Initialize q to have values qvalue and so that it is the independent
  // variable of the problem.
  VectorX<AutoDiffXd> q_autodiff(plant_->num_positions());
  auto q_double = plant_->GetPositions(*context_);
  math::initializeAutoDiff(q_double, q_autodiff);
  plant_autodiff_->GetMutablePositions(context_autodiff_.get()) = q_autodiff;

  const MatrixX<AutoDiffXd> p_EPi_autodiff = p_EPi;
  MatrixX<AutoDiffXd> p_WPi_autodiff(3, kNumPoints);
  MatrixX<AutoDiffXd> Jq_WPi_autodiff(3 * kNumPoints, plant_->num_positions());

  CalcPointsOnEndEffectorAnalyticJacobian(
      *plant_autodiff_, *context_autodiff_,
      p_EPi_autodiff, &p_WPi_autodiff, &Jq_WPi_autodiff);

  // Extract values and derivatives:
  const Matrix3X<double> p_WPi_value =
      math::autoDiffToValueMatrix(p_WPi_autodiff);
  const MatrixX<double> p_WPi_derivs =
      math::autoDiffToGradientMatrix(p_WPi_autodiff);

  // Some sanity checks:
  // Values obtained with <AutoDiffXd> should match those computed with
  // <double>.
  EXPECT_TRUE(CompareMatrices(p_WPi_value, p_WPi,
                              kTolerance, MatrixCompareType::relative));
  // Sizes of the derivatives.
  EXPECT_EQ(p_WPi_derivs.rows(), 3 * kNumPoints);
  EXPECT_EQ(p_WPi_derivs.cols(), plant_->num_positions());

  // Verify the computed Jacobian Jq_WPi matches the one obtained using
  // automatic differentiation.
  EXPECT_TRUE(CompareMatrices(Jq_WPi, p_WPi_derivs,
                              kTolerance, MatrixCompareType::relative));
}

TEST_F(KukaIiwaModelTests, CalcJacobianSpatialVelocity) {
  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();

  SetArbitraryConfiguration();

  // A point P fixed in the end effector frame E.
  Vector3<double> p_EP{0.1, -0.05, 0.02};

  MatrixX<double> Jq_WEp(6, plant_->num_positions());

  // Compute Jacobian using the method under test.
  plant_->CalcJacobianSpatialVelocity(
      *context_, JacobianWrtVariable::kQDot,
      end_effector_link_->body_frame(), p_EP, plant_->world_frame(),
      plant_->world_frame(), &Jq_WEp);

  // Alternatively, compute the Jacobian by taking the gradient of the spatial
  // velocity V_WEp with respect to q̇, since V_WEp = Jq_WEP * q̇. We do that with
  // the steps below.

  // Initialize q̇ to have zero values and so that it is the independent
  // variable of the problem.
  VectorX<AutoDiffXd> qdot_autodiff =
      math::initializeAutoDiff(VectorX<double>::Zero(kNumPositions));
  auto q_double = plant_->GetPositions(*context_);
  VectorX<AutoDiffXd> v_autodiff(kNumVelocities);
  // Update the context with the position values from `context_`.
  plant_autodiff_->SetPositions(context_autodiff_.get(),
                                q_double.cast<AutoDiffXd>());
  // Set the velocity values in the context using q̇.
  plant_autodiff_->MapQDotToVelocity(*context_autodiff_, qdot_autodiff,
                                            &v_autodiff);
  plant_autodiff_->SetVelocities(context_autodiff_.get(), v_autodiff);

  // Compute V_WEp.
  const Body<AutoDiffXd>& end_effector_link_autodiff =
      plant_autodiff_->get_body(end_effector_link_->index());
  const Isometry3<AutoDiffXd> X_WE_autodiff =
      plant_autodiff_->EvalBodyPoseInWorld(*context_autodiff_,
                                           end_effector_link_autodiff);
  const Vector3<AutoDiffXd> p_EP_W =
      X_WE_autodiff.linear() * p_EP.cast<AutoDiffXd>();
  const SpatialVelocity<AutoDiffXd> V_WEp_autodiff =
      plant_autodiff_
          ->EvalBodySpatialVelocityInWorld(*context_autodiff_,
                                           end_effector_link_autodiff)
          .Shift(p_EP_W);

  // Extract the Jacobian generated by automatic differentiation.
  MatrixX<double> Jq_WEp_autodiff =
      math::autoDiffToGradientMatrix(V_WEp_autodiff.get_coeffs());

  // Verify the Jacobian Jq_WEp compupted by the method under test matches the
  // one obtained using automatic differentiation.
  EXPECT_TRUE(CompareMatrices(Jq_WEp, Jq_WEp_autodiff, kTolerance,
                              MatrixCompareType::relative));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
