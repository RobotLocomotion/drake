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

using Eigen::Vector3d;

// For one or more points Ei fixed on a body B, this method computes Jq_v_WEi_W,
// Ei's translational velocity Jacobian in world W with respect to q̇.  However,
// the way this calculation is performed is by forming the partial derivatives
// of p_WoEi_W with respect to q, where p_WoEi_W is Ei's position vector from
// Wo (world origin) expressed in W (world frame).
void CalcJacobianViaPartialDerivativesOfPositionWithRespectToQ(
    const MultibodyPlant<AutoDiffXd>& plant,
    systems::Context<AutoDiffXd>* context,
    const VectorX<double>& q_double,
    const Frame<AutoDiffXd>& frame_E,
    const Matrix3X<double>& p_EoEi_E_double,
    MatrixX<double>* p_WoEi_W_deriv_wrt_q) {
  const int num_positions = plant.num_positions();
  const int kNumPoints = p_EoEi_E_double.cols();
  DRAKE_THROW_UNLESS(p_WoEi_W_deriv_wrt_q != nullptr);
  EXPECT_EQ(p_WoEi_W_deriv_wrt_q->rows(), 3 * kNumPoints);
  EXPECT_EQ(p_WoEi_W_deriv_wrt_q->cols(), num_positions);
  EXPECT_EQ(q_double.rows(), num_positions);
  EXPECT_EQ(p_EoEi_E_double.rows(), 3);

  // Initialize an array of variables q for subsequent partial differentiation
  // and set q's values to match this system's generalized coordinate values.
  VectorX<AutoDiffXd> q(num_positions);
  math::initializeAutoDiff(q_double, q);
  plant.GetMutablePositions(context) = q;

  // Reserve space and then for each point Ei, calculate Ei's position from
  // Wo (World origin), expressed in world W.
  const Matrix3X<AutoDiffXd> p_EoEi_E = p_EoEi_E_double;
  Matrix3X<AutoDiffXd> p_WoEi_W(3, kNumPoints);

  // Create shortcuts to end-effector link frame E and world frame W.
  const Frame<AutoDiffXd>& frame_W = plant.world_frame();
  plant.CalcPointsPositions(*context, frame_E,  p_EoEi_E,   /* From frame E */
                                      frame_W, &p_WoEi_W);  /* To frame W */

  // Form the partial derivatives of p_WoEi_W with respect to q,
  // evaluated at q's values.
  *p_WoEi_W_deriv_wrt_q = math::autoDiffToGradientMatrix(p_WoEi_W);

  // Ensure proper sizes for the matrix storing the partial derivatives.
  EXPECT_EQ(p_WoEi_W_deriv_wrt_q->rows(), 3 * kNumPoints);
  EXPECT_EQ(p_WoEi_W_deriv_wrt_q->cols(), num_positions);
}

TEST_F(KukaIiwaModelTests, FixtureInvariants) {
  // Sanity check basic invariants.
  // Seven dofs for the arm plus floating base.
  EXPECT_EQ(plant_->num_joints(), kNumJoints);
  EXPECT_EQ(plant_->num_positions(), kNumPositions);
  EXPECT_EQ(plant_->num_velocities(), kNumVelocities);
}

// This unit test verifies the calculation of a set of points' translational
// Jacobian with respect to q̇ when the context stores a non-unit quaternion.
TEST_F(KukaIiwaModelTests, CalcJacobianTranslationalVelocityNonUnitQuaternion) {
  SetArbitraryConfiguration(false /* non-unit quaternion for floating base */);

  // A set of points Ei fixed in the end effector frame E.
  const int kNumPoints = 2;  // The set stores 2 points.
  MatrixX<double> p_EoEi_E(3, kNumPoints);
  p_EoEi_E.col(0) << 0.1, -0.05, 0.02;
  p_EoEi_E.col(1) << 0.2, 0.3, -0.15;

  const int num_positions = plant_->num_positions();
  MatrixX<double> p_WoEi_W(3, kNumPoints);
  MatrixX<double> Jq_v_WEi_W(3 * kNumPoints, num_positions);

  const Frame<double>& frame_W = plant_->world_frame();
  const Frame<double>& frame_E = end_effector_link_->body_frame();
  plant_->CalcJacobianTranslationalVelocity(*context_,
                                            JacobianWrtVariable::kQDot,
                                            frame_E,
                                            p_EoEi_E,
                                            frame_W,
                                            frame_W,
                                            &Jq_v_WEi_W);

  // Create shortcuts to end-effector link frame E and world frame W.
  const Body<AutoDiffXd>& end_effector_autodiff =
      plant_autodiff_->get_body(end_effector_link_->index());
  const Frame<AutoDiffXd>& frame_E_autodiff =
      end_effector_autodiff.body_frame();

  // Form the partial derivatives of p_WoEi_W with respect to q,
  // evaluated at q's values.
  const VectorX<double> q_double = plant_->GetPositions(*context_);
  MatrixX<double> p_WoEi_W_deriv_wrt_q(3 * kNumPoints, num_positions);
  CalcJacobianViaPartialDerivativesOfPositionWithRespectToQ(
      *plant_autodiff_, context_autodiff_.get(), q_double, frame_E_autodiff,
      p_EoEi_E, &p_WoEi_W_deriv_wrt_q);

  // Verify the Jacobian Jq_v_WEi_W matches the one from auto-differentiation.
  const double kTolerance = 8 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(Jq_v_WEi_W, p_WoEi_W_deriv_wrt_q,
                              kTolerance, MatrixCompareType::relative));
}

TEST_F(KukaIiwaModelTests, CalcJacobianSpatialVelocity) {
  // Herein, E is the robot's end-effector frame and Ep is a point fixed on E.
  // This test does the following:
  // 1. Calculates Ep's spatial velocity Jacobian with respect to generalized
  //    positions q̇ using the method CalcJacobianSpatialVelocity().
  // 2. Calculates that same quantity, but using autodiff.  Ensure these two
  //    methods produce nearly identical results.
  // 3. Calculate E's angular velocity Jacobian with respect to q̇
  //    using the method CalcJacobianAngularVelocity() and ensure it
  //    produces nearly identical results to the rotational portion of the
  //    aforementioned spatial velocity Jacobian.
  // 4. Calculate Ep's translational velocity Jacobian with respect to q̇
  //    using the method CalcJacobianTranslationalVelocity() and ensure it
  //    produces nearly identical results to the translational portion of the
  //    aforementioned spatial velocity Jacobian.
  // 5. TODO(Mitiguy) Add tests for JacobianWrtVariable::kV

  SetArbitraryConfiguration();

  // Form a position vector from Eo (E's origin) to point Ep, expressed in E.
  Vector3<double> p_EoEp_E{0.1, -0.05, 0.02};

  // For this system which has n generalized positions, compute the 6xn matrix
  // for Ep's spatial velocity Jacobian in W (world).
  const int num_generalized_positions = plant_->num_positions();
  Matrix6X<double> Jq_V_WEp(6, num_generalized_positions);
  const Frame<double>& end_effector_frame = end_effector_link_->body_frame();
  const Frame<double>& world_frame = plant_->world_frame();
  plant_->CalcJacobianSpatialVelocity(*context_, JacobianWrtVariable::kQDot,
      end_effector_frame, p_EoEp_E, world_frame, world_frame, &Jq_V_WEp);

  // Alternately, compute the spatial velocity Jacobian via the gradient of the
  // spatial velocity V_WEp with respect to q̇, since V_WEp = Jq_V_WEp * q̇.
  // We do that with the steps below.

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
  const RotationMatrix<AutoDiffXd>& R_WE_autodiff =
      plant_autodiff_->EvalBodyPoseInWorld(*context_autodiff_,
          end_effector_link_autodiff).rotation();
  const Vector3<AutoDiffXd> p_EoEp_W =
      R_WE_autodiff * p_EoEp_E.cast<AutoDiffXd>();
  const SpatialVelocity<AutoDiffXd> V_WEp_autodiff =
      plant_autodiff_->EvalBodySpatialVelocityInWorld(*context_autodiff_,
                         end_effector_link_autodiff).Shift(p_EoEp_W);

  // Extract the spatial Jacobian generated by automatic differentiation.
  MatrixX<double> Jq_V_WEp_autodiff =
      math::autoDiffToGradientMatrix(V_WEp_autodiff.get_coeffs());

  // Verify the spatial Jacobian Jq_V_WEp computed by the method under test
  // matches the one obtained using automatic differentiation.
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(Jq_V_WEp, Jq_V_WEp_autodiff, kTolerance,
                              MatrixCompareType::relative));

  // Test CalcJacobianAngularVelocity by calculating the 3xn matrix for
  // E's angular velocity Jacobian in W (world) and ensuring this
  // is the first 3 rows of the spatial velocity Jacobian Jq_V_WEp.
  Matrix3X<double> Jq_w_WE(3, num_generalized_positions);
  plant_->CalcJacobianAngularVelocity(*context_, JacobianWrtVariable::kQDot,
                                      end_effector_frame, world_frame,
                                      world_frame, &Jq_w_WE);

  Matrix3X<double> top_three_rows(3, num_generalized_positions);
  top_three_rows = Jq_V_WEp.template topRows<3>();
  EXPECT_TRUE(CompareMatrices(Jq_w_WE, top_three_rows, kTolerance,
                              MatrixCompareType::relative));

  // Test CalcJacobianTranslationalVelocity() by calculating the 3xn matrix for
  // point Ep's translational velocity Jacobian in W (world) and ensuring this
  // is the last 3 rows of the spatial velocity Jacobian Jq_V_WEp.
  Matrix3X<double> Jq_v_WEp(3, num_generalized_positions);
  plant_->CalcJacobianTranslationalVelocity(*context_,
                                            JacobianWrtVariable::kQDot,
                                            end_effector_frame,
                                            p_EoEp_E,
                                            world_frame,
                                            world_frame,
                                            &Jq_v_WEp);
  Matrix3X<double> bottom_three_rows(3, num_generalized_positions);
  bottom_three_rows = Jq_V_WEp.template bottomRows<3>();
  EXPECT_TRUE(CompareMatrices(Jq_v_WEp, bottom_three_rows, kTolerance,
                              MatrixCompareType::relative));
}

TEST_F(KukaIiwaModelTests, CalcJacobianTranslationalVelocityB) {
  // Form two position vectors, one for each point Ei (i = 1, 2).
  // Each point is regarded as fixed/welded to the end effector frame E.
  const int kNumPoints = 2;
  Matrix3X<double> p_EoEi_E(3, kNumPoints);
  p_EoEi_E.col(0) << 0.1, -0.05, 0.02;  // Position from Eo to first point.
  p_EoEi_E.col(1) << 0.2, 0.3, -0.15;   // Position from Eo to second point.

  // Make space for stacked Jacobians with respect to generalized coordinates.
  // Jq_v_WEi_W stores points Ei's translational velocity Jacobian in W (world).
  const int num_positions = plant_->num_positions();
  MatrixX<double> Jq_v_WEi_W(3 * kNumPoints, num_positions);

  // Set arbitrary joint angles.
  SetArbitraryConfiguration();

  // Calculate the 6xn matrix Jq_v_WEi_W that stores each of the two points Ei's
  // translational velocity Jacobian in world W, with respect to q̇.
  const Frame<double>& frame_E = end_effector_link_->body_frame();
  const Frame<double>& frame_W = plant_->world_frame();
  plant_->CalcJacobianTranslationalVelocity(*context_,
                                            JacobianWrtVariable::kQDot,
                                            frame_E,
                                            p_EoEi_E,
                                            frame_W,
                                            frame_W,
                                            &Jq_v_WEi_W);

  // Create shortcuts to end-effector link frame E and world frame W.
  const Body<AutoDiffXd>& end_effector_autodiff =
      plant_autodiff_->get_body(end_effector_link_->index());
  const Frame<AutoDiffXd>& frame_E_autodiff =
      end_effector_autodiff.body_frame();

  // Form the partial derivatives of p_WoEi_W with respect to q,
  // evaluated at q's values.
  const VectorX<double> q_double = plant_->GetPositions(*context_);
  MatrixX<double> p_WoEi_W_deriv_wrt_q(3 * kNumPoints, num_positions);
  CalcJacobianViaPartialDerivativesOfPositionWithRespectToQ(
      *plant_autodiff_, context_autodiff_.get(), q_double, frame_E_autodiff,
      p_EoEi_E, &p_WoEi_W_deriv_wrt_q);

  // Ensure proper sizes for the matrix storing the partial derivatives.
  EXPECT_EQ(p_WoEi_W_deriv_wrt_q.rows(), 3 * kNumPoints);
  EXPECT_EQ(p_WoEi_W_deriv_wrt_q.cols(), num_positions);

  // Verify the Jacobian Jq_v_WEi_W matches the one from auto-differentiation.
  const double kTolerance = 8 * std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(CompareMatrices(Jq_v_WEi_W, p_WoEi_W_deriv_wrt_q,
                              kTolerance, MatrixCompareType::relative));
}

// Fixture to setup a simple 2-link pendulum MBP model with z-axis pin
// joints. The model is in the x-y plane and is configured
// to have both links parallel to the x-axis.
// Points Wo B1o Fo Mo B2o are sequentially along a line parallel to 𝐖𝐱.
//
class TwoDOFPlanarPendulumTest : public ::testing::Test {
 public:
  // Setup the MBP.
  void SetUp() override {
    // Spatial inertia for each body. The inertia values are not important
    // because these are only testing CenterOfMass Jacobian methods.
    const SpatialInertia<double> M_B =
        SpatialInertia<double>::MakeFromCentralInertia(
            mass_, Vector3<double>::Zero(),
            0.0 * UnitInertia<double>::SolidBox(1.0, 1.0, 1.0));

    // Create an empty MultibodyPlant.
    plant_ = std::make_unique<MultibodyPlant<double>>(0.0);

    body1_ = &plant_->AddRigidBody("Body1", M_B);
    body2_ = &plant_->AddRigidBody("Body2", M_B);

    joint1_ = &plant_->AddJoint<RevoluteJoint>(
        "PinJoint1", plant_->world_body(), std::nullopt, *body1_, X_B1W_,
        Vector3d::UnitZ());
    joint2_ = &plant_->AddJoint<RevoluteJoint>(
        "PinJoint2", *body1_, X_B1F_, *body2_, X_B2M_, Vector3d::UnitZ());

    plant_->Finalize();
    // Create a context to store the state for this tree:
    context_ = plant_->CreateDefaultContext();
  }

  const internal::MultibodyTree<double>& tree() const {
    return internal::GetInternalTree(*plant_);
  }

 protected:
  const double kTolerance = 10 * std::numeric_limits<double>::epsilon();
  const double mass_ = 5.0;         // kg
  const double link_length_ = 4.0;  // meters
  const double wz1_ = 3.0;          // rad/sec
  const double wz2_ = 1.2;          // rad/sec

  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<Context<double>> context_;
  const RigidBody<double>* body1_{nullptr};
  const RigidBody<double>* body2_{nullptr};
  const RevoluteJoint<double>* joint1_{nullptr};
  const RevoluteJoint<double>* joint2_{nullptr};
  math::RigidTransformd X_B1W_{Vector3d(-0.5 * link_length_, 0.0, 0.0)};
  math::RigidTransformd X_B1F_{Vector3d(0.5 * link_length_, 0.0, 0.0)};
  math::RigidTransformd X_B2M_{Vector3d(-0.5 * link_length_, 0.0, 0.0)};
};

TEST_F(TwoDOFPlanarPendulumTest,
       CalcJacobianVelocityAndBiasAccelerationOfSystemCenterOfMass) {
  Eigen::VectorXd state = Eigen::Vector4d(0.0, 0.0, wz1_, wz2_);
  joint1_->set_angle(context_.get(), state[0]);
  joint2_->set_angle(context_.get(), state[1]);
  joint1_->set_angular_rate(context_.get(), state[2]);
  joint2_->set_angular_rate(context_.get(), state[3]);

  // Test for CalcJacobianCenterOfMassTranslationalVelocity()
  Eigen::MatrixXd Js_v_WCcm_W(3, plant_->num_velocities());
  plant_->CalcJacobianCenterOfMassTranslationalVelocity(
      *context_, JacobianWrtVariable::kV, plant_->world_frame(),
      plant_->world_frame(), &Js_v_WCcm_W);

  Eigen::MatrixXd Js_v_WCcm_W_expected(3, plant_->num_velocities());
  // CCm's velocity in world W is expected to be (L wz1_ + 0.25 L wz2_) 𝐖𝐲,
  // hence the CCm's translational Jacobian with respect to {wz1_ , wz2_} is
  // { L 𝐖𝐲, 0.25 L 𝐖𝐲 } = { [0, L, 0] [0, 0.25 L, 0] }
  Js_v_WCcm_W_expected << 0.0, 0.0,
                          link_length_, 0.25 * link_length_,
                          0.0, 0.0;
  Vector3d v_WCcm_W_expected =
      (wz1_ * link_length_ + wz2_ * 0.25 * link_length_) * Vector3d::UnitY();

  EXPECT_TRUE(CompareMatrices(Js_v_WCcm_W, Js_v_WCcm_W_expected, kTolerance));
  EXPECT_TRUE(
      CompareMatrices(Js_v_WCcm_W * state.tail(plant_->num_velocities()),
                      v_WCcm_W_expected, kTolerance));

  // Test for CalcBiasCenterOfMassTranslationalAcceleration()
  const Vector3<double>& abias_WCcm_W =
      plant_->CalcBiasCenterOfMassTranslationalAcceleration(
          *context_, JacobianWrtVariable::kV, plant_->world_frame(),
          plant_->world_frame());

  // CCm's bias translational in world W is expected to be
  // abias_WCcm = -L (wz1_² + 0.5 wz1_ wz2_ + 0.25 wz2_²) 𝐖𝐱
  Vector3d abias_WCcm_W_expected =
      -link_length_ * (wz1_ * wz1_ + 0.5 * wz1_ * wz2_ + 0.25 * wz2_ * wz2_) *
      Vector3d::UnitX();
  EXPECT_TRUE(CompareMatrices(abias_WCcm_W, abias_WCcm_W_expected, kTolerance));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
