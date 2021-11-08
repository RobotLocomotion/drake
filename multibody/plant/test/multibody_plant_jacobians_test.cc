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

using Eigen::MatrixXd;
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
  math::InitializeAutoDiff(q_double, &q);
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
  *p_WoEi_W_deriv_wrt_q = math::ExtractGradient(p_WoEi_W);

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
      math::InitializeAutoDiff(VectorX<double>::Zero(kNumPositions));
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
      math::ExtractGradient(V_WEp_autodiff.get_coeffs());

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

// Fixture for a two degree-of-freedom pendulum having two links A and B.
// Link A is connected to world (frame W) with a z-axis pin joint (PinJoint1).
// Link B is connected to link A with another z-axis pin joint (PinJoint2).
// Hence links A and B only move in the world's x-y plane (perpendicular to Wz).
// The long axis of link A is parallel to A's unit vector Ax and
// the long axis of link B is parallel to B's unit vector Bx.
// PinJoint1 and PinJoint2 are located at distal ends of the links.
// PinJoint1 is collocated with Wo (world frame origin)
// PinJoint2 is collocated with Fo (frame F's origin) and Mo (frame M's origin)
// where frame F is fixed/welded to link A and frame M is fixed to link B.
// In the baseline configuration, the origin points Wo Ao Fo Mo Bo are
// sequential along the links (they form a line parallel to Wx = Ax = Bx).
class TwoDOFPlanarPendulumTest : public ::testing::Test {
 public:
  // Setup the MBP.
  void SetUp() override {
    // Set a spatial inertia for each link.  For now, these are unimportant
    // because this fixture is only used for kinematic tests (e.g., Jacobians).
    const UnitInertia<double> G_Bcm =
        UnitInertia<double>::SolidBox(link_length_, 1, 1);
    const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
    const SpatialInertia<double> M_Bcm(mass_link_, p_BoBcm_B, G_Bcm);

    // Create an empty MultibodyPlant and then add the two links.
    plant_ = std::make_unique<MultibodyPlant<double>>(0.0);
    bodyA_instance_ = plant_->AddModelInstance("bodyAInstanceName");
    bodyB_instance_ = plant_->AddModelInstance("bodyBInstanceName");
    bodyA_ = &plant_->AddRigidBody("bodyA", bodyA_instance_, M_Bcm);
    bodyB_ = &plant_->AddRigidBody("bodyB", bodyB_instance_, M_Bcm);

    // Create a revolute joint that connects point Wo of the world frame to a
    // unnamed point of link A that is a distance of link_length/2 from link A's
    // centroid (point Ao).
    const Vector3d p_AoWo_A(-0.5 * link_length_, 0.0, 0.0);
    joint1_ = &plant_->AddJoint<RevoluteJoint>("PinJoint1",
        plant_->world_body(), std::nullopt,
        *bodyA_, math::RigidTransformd(p_AoWo_A), Vector3d::UnitZ());

    // Create a revolute joint that connects point Fo (frame F's origin) to
    // point Mo (frame M's origin), where frame F is fixed/welded to the
    // distal end of link A (Fo is a distance of link_length/2 from Ao) and
    // frame M is fixed/welded to link B.  Mo is a distance of link_length/2
    // from link B's centroid (point Bo).
    const Vector3d p_AoFo_A(0.5 * link_length_, 0.0, 0.0);
    const Vector3d p_BoMo_B(-0.5 * link_length_, 0.0, 0.0);
    joint2_ = &plant_->AddJoint<RevoluteJoint>("PinJoint2",
        *bodyA_, math::RigidTransformd(p_AoFo_A),
        *bodyB_, math::RigidTransformd(p_BoMo_B), Vector3d::UnitZ());

    // Finalize the plant and create a context to store this plant's state.
    plant_->Finalize();
    context_ = plant_->CreateDefaultContext();
  }

  const RigidBody<double>& rigid_bodyA() const {
    DRAKE_ASSERT(bodyA_ != nullptr);
    return *bodyA_;
  }
  const RigidBody<double>& rigid_bodyB() const {
    DRAKE_ASSERT(bodyB_ != nullptr);
    return *bodyB_;
  }

 protected:
  // Since the maximum absolute value of acceleration in this test is
  // approximately ω² * (2 * link_length) ≈ 72 , we test that the errors in
  // acceleration calculations are less than 3 bits (2^3 = 8).
  const double kTolerance = 8 * std::numeric_limits<double>::epsilon();
  const double mass_link_ = 5.0;    // kg
  const double link_length_ = 4.0;  // meters
  const double wAz_ = 3.0;          // rad/sec
  const double wBz_ = 2.0;          // rad/sec

  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<Context<double>> context_;
  const RigidBody<double>* bodyA_{nullptr};
  const RigidBody<double>* bodyB_{nullptr};
  const RevoluteJoint<double>* joint1_{nullptr};
  const RevoluteJoint<double>* joint2_{nullptr};
  ModelInstanceIndex bodyA_instance_;
  ModelInstanceIndex bodyB_instance_;
};

TEST_F(TwoDOFPlanarPendulumTest, CalcBiasAccelerations) {
  Eigen::VectorXd state = Eigen::Vector4d(0.0, 0.0, wAz_, wBz_);
  joint1_->set_angle(context_.get(), state[0]);
  joint2_->set_angle(context_.get(), state[1]);
  joint1_->set_angular_rate(context_.get(), state[2]);
  joint2_->set_angular_rate(context_.get(), state[3]);

  // Point Ap is the point of A located at the revolute joint connecting link A
  // and link B.  Calculate Ap's bias spatial acceleration in world W.
  const Frame<double>& frame_A = bodyA_->body_frame();
  const Frame<double>& frame_W = plant_->world_frame();
  const Vector3<double> p_AoAp_A(0.5 * link_length_, 0.0, 0.0);
  const SpatialAcceleration<double> aBias_WAp_W =
      plant_->CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
          frame_A, p_AoAp_A, frame_W, frame_W);

  // Simple by-hand analysis gives aBias_WAp_W = -L wAz_² Wx.
  const double wA_squared = wAz_ * wAz_;
  const Vector3<double> aBias_WAp_W_expected = -link_length_ *
      wA_squared * Vector3d::UnitX();
  EXPECT_TRUE(CompareMatrices(aBias_WAp_W.translational(), aBias_WAp_W_expected,
                              kTolerance));
  EXPECT_TRUE(CompareMatrices(aBias_WAp_W.rotational(), Vector3d::Zero(),
                              kTolerance));

  // Point Bp is the point of B located at the most distal end of link B.
  // Calculate Bp's bias spatial acceleration in world W.
  const Frame<double>& frame_B = bodyB_->body_frame();
  const Vector3<double> p_BoBp_B(0.5 * link_length_, 0.0, 0.0);
  const SpatialAcceleration<double> aBias_WBp_W =
      plant_->CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                          frame_B, p_BoBp_B, frame_W, frame_W);

  // By-hand analysis gives aBias_WBp_W = -L wAz_² Wx - L (wAz_ + wBz_)² Wx
  const double wAB_squared = (wAz_ + wBz_) * (wAz_ + wBz_);
  const Vector3<double> aBias_WBp_W_expected = (-link_length_ * wA_squared +
      -link_length_ * wAB_squared) * Vector3d::UnitX();
  EXPECT_TRUE(CompareMatrices(aBias_WBp_W.translational(), aBias_WBp_W_expected,
                              kTolerance));
  EXPECT_TRUE(CompareMatrices(aBias_WBp_W.rotational(), Vector3d::Zero(),
                              kTolerance));

  // For point Ap of A, calculate Ap's bias spatial acceleration in frame A.
  // Simple by-hand analysis gives aBias_AAp_A = Vector3d::Zero().
  const SpatialAcceleration<double> aBias_AAp_A =
      plant_->CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                          frame_A, p_AoAp_A, frame_A, frame_A);
  EXPECT_TRUE(CompareMatrices(aBias_AAp_A.translational(), Vector3d::Zero(),
      kTolerance));
  EXPECT_TRUE(CompareMatrices(aBias_AAp_A.rotational(), Vector3d::Zero(),
      kTolerance));

  // For point Bp of B, calculate Bp's bias spatial acceleration in frame A.
  const SpatialAcceleration<double> aBias_ABp_A =
      plant_->CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                          frame_B, p_BoBp_B, frame_A, frame_A);

  // By-hand analysis gives aBias_ABp_A = -L wBz_² Ax.
  const Vector3<double> aBias_ABp_A_expected =
      -link_length_ * wBz_ * wBz_ * Vector3d::UnitX();
  EXPECT_TRUE(CompareMatrices(aBias_ABp_A.translational(), aBias_ABp_A_expected,
                              kTolerance));
  EXPECT_TRUE(CompareMatrices(aBias_ABp_A.rotational(), Vector3d::Zero(),
                              kTolerance));

  // Points Bi (i = 0, 1) are fixed to link B and located from Bo as below.
  const int kNumPoints = 2;  // The set stores 2 points.
  Matrix3X<double> p_BoBi_B(3, kNumPoints);
  const double x = 0.1, y = -0.2, z = 0.3;
  p_BoBi_B.col(0) << x, y, z;
  p_BoBi_B.col(1) << 2 * x, 3 * y, 4 * z;

  // Compute Bi's bias translational acceleration with respect to generalized
  // velocities v measured in frame A, expressed in frame B.
  Matrix3X<double> aBias_ABi_B = plant_->CalcBiasTranslationalAcceleration(
      *context_, JacobianWrtVariable::kV, frame_B, p_BoBi_B, frame_A, frame_B);

  // MotionGenesis gives  aBias_AB0_B = -0.5 (L+2*x) wBz_² Bx -   y wBz_² By.
  // MotionGenesis gives  aBias_AB1_B = -0.5 (L+4*x) wBz_² Bx - 3 y wBz_² By.
  const double wB_squared = wBz_ * wBz_;
  const Vector3<double> aBias_AB0_B_expected(
      -0.5 * (link_length_ + 2 * x) * wB_squared, -y * wB_squared, 0);
  const Vector3<double> aBias_AB1_B_expected(
      -0.5 * (link_length_ + 4 * x) * wB_squared, -3 * y * wB_squared, 0);
  EXPECT_TRUE(CompareMatrices(aBias_ABi_B.col(0), aBias_AB0_B_expected,
                              kTolerance));
  EXPECT_TRUE(CompareMatrices(aBias_ABi_B.col(1), aBias_AB1_B_expected,
                              kTolerance));
}

TEST_F(TwoDOFPlanarPendulumTest,
       CalcJacobianVelocityAndBiasAccelerationEtcOfSystemCenterOfMass) {
  Eigen::VectorXd state = Eigen::Vector4d(0.0, 0.0, wAz_, wBz_);
  joint1_->set_angle(context_.get(), state[0]);
  joint2_->set_angle(context_.get(), state[1]);
  joint1_->set_angular_rate(context_.get(), state[2]);
  joint2_->set_angular_rate(context_.get(), state[3]);

  // Shortcuts to rigid bodies.
  const RigidBody<double>& body_A = rigid_bodyA();
  const RigidBody<double>& body_B = rigid_bodyB();

  // Verify Body::CalcCenterOfMassTranslationalVelocityInWorld() with by-hand
  // results for translational velocities measured in the world frame W:
  // Acm's translational velocity: v_WAcm_W = 0.5 L wAz_ Wy.
  // Bcm's translational velocity: v_WBcm_W = (0.5 L wBz_ + 1.5 L wAz_) Wy.
  const Vector3d v_WAcm_W_expected(0, 0.5 * link_length_ * wAz_, 0);
  const Vector3d v_WBcm_W_expected(0, 0.5 * link_length_ * wBz_ +
                                      1.5 * link_length_ * wAz_, 0);
  const Vector3d v_WAcm_W = body_A.CalcCenterOfMassTranslationalVelocityInWorld(
      *context_);
  const Vector3d v_WBcm_W = body_B.CalcCenterOfMassTranslationalVelocityInWorld(
      *context_);
  EXPECT_TRUE(CompareMatrices(v_WAcm_W, v_WAcm_W_expected, kTolerance));
  EXPECT_TRUE(CompareMatrices(v_WBcm_W, v_WBcm_W_expected, kTolerance));

  // Verify MultibodyPlant::CalcCenterOfMassTranslationalVelocityInWorld().
  const Vector3d v_WScm_W =
      plant_->CalcCenterOfMassTranslationalVelocityInWorld(*context_);
  const Vector3d v_WScm_W_expected =
      (wAz_ * link_length_ + wBz_ * 0.25 * link_length_) * Vector3d::UnitY();
  EXPECT_TRUE(CompareMatrices(v_WScm_W, v_WScm_W_expected, kTolerance));
  const Vector3d v_WScm_W_alternate = (mass_link_ * v_WAcm_W_expected
                                    +  mass_link_ * v_WBcm_W_expected)
                                    / (2 * mass_link_);
  EXPECT_TRUE(CompareMatrices(v_WScm_W, v_WScm_W_alternate, kTolerance));

  // Test CalcJacobianCenterOfMassTranslationalVelocity() for full MBP.
  const int num_velocities = plant_->num_velocities();
  const Frame<double>& frame_W = plant_->world_frame();
  Eigen::MatrixXd Js_v_WScm_W(3, num_velocities);
  plant_->CalcJacobianCenterOfMassTranslationalVelocity(
      *context_, JacobianWrtVariable::kV, frame_W, frame_W, &Js_v_WScm_W);

  Eigen::MatrixXd Js_v_WScm_W_expected(3, num_velocities);
  // Denoting Scm as the center of mass of the system formed by links A and B,
  // Scm's velocity in world W is expected to be (L wAz_ + 0.25 L wBz_) Wy,
  // hence Scm's translational Jacobian with respect to {wAz_ , wBz_} is
  // { L Wy, 0.25 L Wy } = { [0, L, 0] [0, 0.25 L, 0] }
  Js_v_WScm_W_expected << 0.0, 0.0,
                          link_length_, 0.25 * link_length_,
                          0.0, 0.0;
  EXPECT_TRUE(CompareMatrices(Js_v_WScm_W, Js_v_WScm_W_expected, kTolerance));
  EXPECT_TRUE(
      CompareMatrices(Js_v_WScm_W * state.tail(num_velocities),
                      v_WScm_W_expected, kTolerance));

  // Test CalcJacobianCenterOfMassTranslationalVelocity() for 1 model instance.
  std::vector<ModelInstanceIndex> model_instances;
  const ModelInstanceIndex bodyA_instance_index =
      plant_->GetModelInstanceByName("bodyAInstanceName");
  model_instances.push_back(bodyA_instance_index);
  Eigen::MatrixXd Js_v_WAcm_W(3, num_velocities);
  plant_->CalcJacobianCenterOfMassTranslationalVelocity(*context_,
      model_instances, JacobianWrtVariable::kV, frame_W, frame_W, &Js_v_WAcm_W);

  // Acm's velocity in world W is expected to be 0.5 L wAz_ Wy,
  // hence Acm's translational Jacobian with respect to {wAz_ , wBz_} is
  // { 0.5 L Wy, 0 } = { [0, 0.5 L, 0] [0, 0, 0] }
  Eigen::MatrixXd Js_v_WAcm_W_expected(3, num_velocities);
  Js_v_WAcm_W_expected << 0.0, 0.0,
                          0.5 * link_length_, 0.0,
                          0.0, 0.0;
  EXPECT_TRUE(CompareMatrices(Js_v_WAcm_W, Js_v_WAcm_W_expected, kTolerance));
  EXPECT_TRUE(
      CompareMatrices(Js_v_WAcm_W * state.tail(num_velocities),
                      v_WAcm_W_expected, kTolerance));

  // Test CalcJacobianCenterOfMassTranslationalVelocity() for 2 model instances.
  // This should produce the same results as Scm (system center of mass).
  const ModelInstanceIndex bodyB_instance_index =
      plant_->GetModelInstanceByName("bodyBInstanceName");
  model_instances.push_back(bodyB_instance_index);
  plant_->CalcJacobianCenterOfMassTranslationalVelocity(*context_,
      model_instances, JacobianWrtVariable::kV, frame_W, frame_W, &Js_v_WScm_W);
  EXPECT_TRUE(CompareMatrices(Js_v_WAcm_W, Js_v_WAcm_W_expected, kTolerance));

  // Test for CalcBiasCenterOfMassTranslationalAcceleration().
  const Vector3<double>& abias_WScm_W =
      plant_->CalcBiasCenterOfMassTranslationalAcceleration(
          *context_, JacobianWrtVariable::kV, plant_->world_frame(),
          plant_->world_frame());

  // Scm's bias translational in world W is expected to be
  // abias_WScm = -L (wAz_² + 0.5 wAz_ wBz_ + 0.25 wBz_²) 𝐖𝐱
  Vector3d abias_WScm_W_expected =
      -link_length_ * (wAz_ * wAz_ + 0.5 * wAz_ * wBz_ + 0.25 * wBz_ * wBz_) *
      Vector3d::UnitX();
  EXPECT_TRUE(CompareMatrices(abias_WScm_W, abias_WScm_W_expected, kTolerance));
}

// Fixture for two degree-of-freedom 3D satellite tracker with bodies A and B.
// Body A is a cylinder connected to world (frame W) with a y-axis pin joint
// (PinJoint1).  Body B is a parabolic satellite dish connected to body A with
// a z-axis pin joint (PinJoint2).  Orthogonal unit vectors Wx, Wy, Wz are fixed
// in frame W (on Earth's surface) with Wx locally North and Wy vertically
// upward and parallel to PinJoint1.  Sets of orthogonal unit vectors Ax, Ay, Az
// and Bx, By, Bz are fixed in bodies A and B, respectively.  Initially, Aᵢ = Wᵢ
// (i = x, y, z) and then A is subjected to a right-handed rotation of -qA Wy.
// Similarly, initially, Bᵢ = Aᵢ (i = x, y, z) and then B is subjected to a
// right-handed rotation of qB Az.
// PinJoint1 is collocated with Wo (frame W's origin) and Ao (body A's origin).
// PinJoint2 is collocated with Ao (frame A's origin) and Bo (body B's origin).
// Hence PinJoint1 and PinJoint2 do not translate relative to world W.
// The focal point of satellite dish is point Q, located L Bx from Bo.
class SatelliteTrackerTest : public ::testing::Test {
 public:
  // Setup the Multibody Plant.
  void SetUp() override {
    // Set a spatial inertia for each body.  For now, these are unimportant
    // because this fixture is only used for kinematic tests.
    const double mA = 4;              // mass of cylinder A (kg).
    const double rA = 0.2, LA = 0.5;  // cylinder A's radius and length (meter).
    const UnitInertia<double> G_Acm =
        UnitInertia<double>::SolidCylinder(rA, LA);
    const Vector3d p_AoAcm_A = Vector3d::Zero();
    const SpatialInertia<double> M_Acm(mA, p_AoAcm_A, G_Acm);

    // Create an empty MultibodyPlant and then add the two bodies.
    plant_ = std::make_unique<MultibodyPlant<double>>(0.0);
    bodyA_ = &plant_->AddRigidBody("BodyA", M_Acm);
    bodyB_ = &plant_->AddRigidBody("BodyB", M_Acm);  // same as bodyA_.

    // Create a pin (revolute) joint that connects point Wo to point Ao.
    // Described above: The angle associated with this pin joint is -qA Wy.
    const Vector3d p_WoAo_W(0, 0, 0);  // Points Wo and Ao are collocated.
    const Vector3d p_AoWo_A(0, 0, 0);
    joint1_ = &plant_->AddJoint<RevoluteJoint>("PinJoint1",
        plant_->world_body(), math::RigidTransformd(p_WoAo_W),
        *bodyA_, math::RigidTransformd(p_AoWo_A), -Vector3d::UnitY());

    // Create a pin (revolute) joint that connects point Ao to point Bo.
    // Described above: The angle associated with this pin joint is qB Az.
    const Vector3d p_AoBo_A(0, 0, 0);  // Points Ao and Bo are collocated.
    const Vector3d p_BoAo_A(0, 0, 0);
    joint2_ = &plant_->AddJoint<RevoluteJoint>("PinJoint2",
        *bodyA_, math::RigidTransformd(p_AoBo_A),
        *bodyB_, math::RigidTransformd(p_BoAo_A), Vector3d::UnitZ());

    // Finalize the plant and create a context to store this plant's state.
    plant_->Finalize();
    context_ = plant_->CreateDefaultContext();
  }

 protected:
  // Since the maximum absolute value of translation acceleration in this test
  // is approximately ω² * LB_ ≈ 0.5² * 0.6 = 0.15 (which is larger than the
  // maximum absolute value of angular acceleration of 0.12), we test that the
  // errors in bias acceleration calculations are less than 3 bits (2^3 = 8).
  // In connection with real numbers whose absolute value are < 0.25, double-
  // precision calculations should be accurate to ≈ machine epsilon / 4.
  // So, for this test, kTolerance = 2^3 * machine_epsilon / 4.
  const double kTolerance = 2 * std::numeric_limits<double>::epsilon();
  const double LB_ = 0.5;  // Bx measure of Q's position vector from Bo (meter).
  const double qB_ = 30 * M_PI / 180.0;  // rad.
  const double wA_ = 0.3;  // rad/sec.  Note: wA_ is the time derivative of qA.
  const double wB_ = 0.4;  // rad/sec.  Note: wB_ is the time derivative of qB.

  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<Context<double>> context_;
  const RigidBody<double>* bodyA_{nullptr};
  const RigidBody<double>* bodyB_{nullptr};
  const RevoluteJoint<double>* joint1_{nullptr};
  const RevoluteJoint<double>* joint2_{nullptr};
};

TEST_F(SatelliteTrackerTest, CalcBiasAccelerations) {
  // MotionGenesis analytical results for this system are:
  // αBias_AB = 0
  // αBias_WB = -cos(qB) wA wB Bx  +  sin(qB) wA wB By
  // aBias_AQ = -L wB^2 Bx
  // aBias_WQ = -L (wB^2 + cos(qB)^2 wA^2) Bx
  //           + L sin(qB) cos(qB) wA^2 By
  //           - 2 L sin(qB) wA wB Bz
  const double cosqB = std::cos(qB_);
  const double sinqB = std::sin(qB_);
  const double wAA = wA_ * wA_;
  const double wAB = wA_ * wB_;
  const double wBB = wB_ * wB_;
  const Vector3d alphaBias_AB_B_expected(0, 0, 0);
  const Vector3d alphaBias_WB_B_expected(-cosqB * wAB, sinqB * wAB, 0);
  const Vector3d aBias_AQ_B_expected(-LB_ * wBB, 0, 0);
  const Vector3d aBias_WQ_B_expected = LB_ * Vector3d(
    -wBB - cosqB * cosqB * wAA, sinqB * cosqB * wAA, -2 * sinqB * wAB);

  // Use Drake to calculate the same quantities.
  // Note: The angle qA is irrelevant because Ao is stationary in the world
  // frame W, i.e., Ao's translational velocity and acceleration in W is 0.
  // The angular rate wA_ is relevant since A's angular velocity in W is -wA Wy
  // and this affects B's bias angular acceleration in W as well as Q's bias
  // translational acceleration in W.  No forces are relevant because bias
  // accelerations are only a function of state(q, v) and calculations in this
  // test depend only on current state (not a future state subject to F = ma).
  const double qA_irrelevant = 15 * M_PI / 180;  // radians.
  Eigen::VectorXd state = Eigen::Vector4d(qA_irrelevant, qB_, wA_, wB_);
  joint1_->set_angle(context_.get(), state[0]);
  joint2_->set_angle(context_.get(), state[1]);
  joint1_->set_angular_rate(context_.get(), state[2]);
  joint2_->set_angular_rate(context_.get(), state[3]);

  // Calculate point Q's bias spatial acceleration in world W, expressed in B.
  // Note: Point Q is B's focal point and Q's position vector from Bo is LB_ Bx.
  const Frame<double>& frame_B = bodyB_->body_frame();
  const Frame<double>& frame_W = plant_->world_frame();
  const Vector3d p_BoQ_B(LB_, 0, 0);
  const SpatialAcceleration<double> aBias_WQ_B =
      plant_->CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                          frame_B, p_BoQ_B, frame_W, frame_B);
  EXPECT_TRUE(CompareMatrices(aBias_WQ_B.rotational(), alphaBias_WB_B_expected,
                              kTolerance));
  EXPECT_TRUE(CompareMatrices(aBias_WQ_B.translational(), aBias_WQ_B_expected,
                              kTolerance));

  // Calculate Q's bias spatial acceleration in frame A, expressed in frame B.
  const Frame<double>& frame_A = bodyA_->body_frame();
  const SpatialAcceleration<double> aBias_AQ_B =
      plant_->CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                          frame_B, p_BoQ_B, frame_A, frame_B);
  EXPECT_TRUE(CompareMatrices(aBias_AQ_B.rotational(), alphaBias_AB_B_expected,
                              kTolerance));
  EXPECT_TRUE(CompareMatrices(aBias_AQ_B.translational(), aBias_AQ_B_expected,
                              kTolerance));

  // Point Bp is fixed to link B and is located from Bo (B's origin) as below.
  // Compute Bp's bias translational acceleration with respect to generalized
  // velocities v measured in frame A, expressed in frame B.
  const double x = 0.1, y = -0.2, z = 0.3;
  Vector3<double> p_BoBp_B(x, y, z);
  Vector3<double> aBias_ABp_B = plant_->CalcBiasTranslationalAcceleration(
      *context_, JacobianWrtVariable::kV, frame_B, p_BoBp_B, frame_A, frame_B);

  // MotionGenesis gives  aBias_ABp_B = -x wB_² Bx - y wB_² By.
  const double wB_squared = wB_ * wB_;
  Vector3<double> aBias_ABp_B_expected(-x * wB_squared, -y * wB_squared, 0);
  EXPECT_TRUE(CompareMatrices(aBias_ABp_B, aBias_ABp_B_expected, kTolerance));
}

// This tests the method CalcBiasSpatialAcceleration() against an expected
// solution that uses AutoDiffXd to compute Dt(J𝑠) ⋅ 𝑠, where Dt(J𝑠) is the time
// derivative of the Jacobian with respect to "speeds" 𝑠, and 𝑠 is either
// q̇ (time-derivatives of generalized positions) or v (generalized velocities).
TEST_F(KukaIiwaModelTests, CalcBiasSpatialAcceleration) {
  // Set state to arbitrary non-planar values for the joint's angles and rates.
  SetArbitraryConfiguration();
  const VectorX<double> q = plant_->GetPositions(*context_);
  const VectorX<double> v = plant_->GetVelocities(*context_);
  const int num_positions = plant_->num_positions();
  const int num_velocities = plant_->num_velocities();
  const int num_states = plant_->num_multibody_states();

  // Bias acceleration is not a function of vdot (it is a function of q and v).
  // NaN values for generalized accelerations vdot are chosen to highlight the
  // fact that bias acceleration is not a function of vdot.
  const VectorX<double> vdot = VectorX<double>::Constant(
      num_velocities, std::numeric_limits<double>::quiet_NaN());

  // Enable q_autodiff and v_autodiff to differentiate with respect to time.
  // Note: Pass MatrixXd() so the return gradient uses AutoDiffXd (for which we
  // do have explicit instantiations) instead of AutoDiffScalar<Matrix1d>.
  VectorX<double> qdot(num_positions);
  plant_->MapVelocityToQDot(*context_, v, &qdot);
  auto q_autodiff = math::InitializeAutoDiff(q, MatrixXd(qdot));
  auto v_autodiff = math::InitializeAutoDiff(v, MatrixXd(vdot));

  // Set the context for AutoDiffXd computations.
  VectorX<AutoDiffXd> x_autodiff(num_states);
  x_autodiff << q_autodiff, v_autodiff;
  plant_autodiff_->GetMutablePositionsAndVelocities(context_autodiff_.get()) =
      x_autodiff;

  // Point Ep is affixed/welded to the end-effector E.
  const Vector3<double> p_EEp(0.1, -0.05, 0.02);
  const Vector3<AutoDiffXd> p_EEp_autodiff = p_EEp;

  // Get shortcuts to end-effector link frame E and world frame W.
  const Frame<AutoDiffXd>& frame_E_autodiff =
      plant_autodiff_->get_body(end_effector_link_->index()).body_frame();
  const Frame<AutoDiffXd>& frame_W_autodiff = plant_autodiff_->world_frame();

  // Compute point Ep's spatial velocity Jacobian with respect to generalized
  // velocities v in world frame W, expressed in W, and its time derivative.
  MatrixX<AutoDiffXd> Jv_V_WEp_autodiff(6, num_velocities);
  plant_autodiff_->CalcJacobianSpatialVelocity(
      *context_autodiff_, JacobianWrtVariable::kV, frame_E_autodiff,
      p_EEp_autodiff, frame_W_autodiff, frame_W_autodiff, &Jv_V_WEp_autodiff);

  // Use AutoDiffXd to extract Dt(Jv_V_WEp), the ordinary time-derivative of
  // Ep's spatial Jacobian in world W, expressed in W.
  auto Dt_Jv_V_WEp = math::ExtractGradient(Jv_V_WEp_autodiff);
  Dt_Jv_V_WEp.resize(6, num_velocities);

  // Form the expected bias spatial acceleration via AutoDiffXd results.
  const VectorX<double> AvBias_WEp_expected = Dt_Jv_V_WEp * v;

  // Compute Ep's bias spatial acceleration in world frame W.
  const Frame<double>& frame_W = plant_->world_frame();
  const Frame<double>& frame_E = end_effector_link_->body_frame();
  const SpatialAcceleration<double> AvBias_WEp_W =
      plant_->CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                          frame_E, p_EEp, frame_W, frame_W);

  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 8 * std::numeric_limits<double>::epsilon();

  // Verify computed bias translational acceleration numerical values and ensure
  // the results are stored in a matrix of size (6 x num_velocities).
  EXPECT_TRUE(CompareMatrices(AvBias_WEp_W.get_coeffs(), AvBias_WEp_expected,
                              kTolerance, MatrixCompareType::relative));
}

// This tests the method CalcBiasTranslationalAcceleration() against an expected
// solution that uses AutoDiffXd to compute Dt(J𝑠) ⋅ 𝑠, where Dt(J𝑠) is the
// time derivative of the Jacobian with respect to "speeds" 𝑠, and 𝑠 is either
// q̇ (time-derivatives of generalized positions) or v (generalized velocities).
TEST_F(KukaIiwaModelTests, CalcBiasTranslationalAcceleration) {
  // Set state to arbitrary non-planar values for the joint's angles and rates.
  SetArbitraryConfiguration();

  const int num_velocities = plant_->num_velocities();

  // Bias acceleration is not a function of vdot (it is a function of q and v).
  // NaN values for generalized accelerations vdot are chosen to highlight the
  // fact that bias acceleration is not a function of vdot.
  const VectorX<double> vdot = VectorX<double>::Constant(
      num_velocities, std::numeric_limits<double>::quiet_NaN());

  const VectorX<double> q = plant_->GetPositions(*context_);
  const VectorX<double> v = plant_->GetVelocities(*context_);

  // Enable q_autodiff and v_autodiff to differentiate with respect to time.
  // Note: Pass MatrixXd() so the return gradient uses AutoDiffXd (for which we
  // do have explicit instantiations) instead of AutoDiffScalar<Matrix1d>.
  VectorX<double> qdot(plant_->num_positions());
  plant_->MapVelocityToQDot(*context_, v, &qdot);
  auto q_autodiff = math::InitializeAutoDiff(q, MatrixXd(qdot));
  auto v_autodiff = math::InitializeAutoDiff(v, MatrixXd(vdot));

  // Set the context for AutoDiffXd computations.
  VectorX<AutoDiffXd> x_autodiff(plant_->num_multibody_states());
  x_autodiff << q_autodiff, v_autodiff;
  plant_autodiff_->GetMutablePositionsAndVelocities(context_autodiff_.get()) =
      x_autodiff;

  // Points Ei (i = 0, 1) are affixed/welded to the end-effector E.
  // Designate Ei's positions from origin Eo, expressed in frame E.
  const int kNumPoints = 2;  // The set stores 2 points.
  Matrix3X<double> p_EEi(3, kNumPoints);
  p_EEi.col(0) << 0.1, -0.05, 0.02;
  p_EEi.col(1) << 0.2, 0.3, -0.15;
  const Matrix3X<AutoDiffXd> p_EEi_autodiff = p_EEi;

  // Get shortcuts to end-effector link frame E and world frame W.
  const Frame<AutoDiffXd>& frame_E_autodiff =
      plant_autodiff_->get_body(end_effector_link_->index()).body_frame();
  const Frame<AutoDiffXd>& frame_W_autodiff = plant_autodiff_->world_frame();

  // Compute Ei's translational velocity Jacobian with respect to generalized
  // velocities v in world frame W, expressed in W, and its time derivative.
  MatrixX<AutoDiffXd> Jv_v_WEi_autodiff(3 * kNumPoints, num_velocities);
  plant_autodiff_->CalcJacobianTranslationalVelocity(
      *context_autodiff_, JacobianWrtVariable::kV, frame_E_autodiff,
      p_EEi_autodiff, frame_W_autodiff, frame_W_autodiff, &Jv_v_WEi_autodiff);

  // Use AutoDiffXd to extract Dt(Jv_v_WEi), the ordinary time-derivative of
  // Ei's translational Jacobian in world W, expressed in W.
  auto Dt_Jv_v_WEi = math::ExtractGradient(Jv_v_WEi_autodiff);
  Dt_Jv_v_WEi.resize(3 * kNumPoints, num_velocities);

  // Form the expected bias translational acceleration via AutoDiffXd results.
  const VectorX<double> avBias_WEi_W_expected_VectorX = Dt_Jv_v_WEi * v;

  // Reshape the expected results from VectorX to Matrix3X.
  Matrix3X<double> avBias_WEi_W_expected(3, kNumPoints);
  avBias_WEi_W_expected.col(0) = avBias_WEi_W_expected_VectorX.head(3);
  avBias_WEi_W_expected.col(1) = avBias_WEi_W_expected_VectorX.tail(3);

  // Compute Ep's bias translational acceleration in world frame W.
  const Frame<double>& frame_W = plant_->world_frame();
  const Frame<double>& frame_E = end_effector_link_->body_frame();

  const Matrix3X<double> avBias_WEi_W =
      plant_->CalcBiasTranslationalAcceleration(
          *context_, JacobianWrtVariable::kV, frame_E, p_EEi, frame_W, frame_W);

  // Numerical tolerance used to verify numerical results.
  const double kTolerance = 8 * std::numeric_limits<double>::epsilon();

  // Verify computed bias translational acceleration numerical values and ensure
  // the results are stored in a matrix of size (3 kNumPoints x num_velocities).
  EXPECT_TRUE(CompareMatrices(avBias_WEi_W, avBias_WEi_W_expected, kTolerance,
                              MatrixCompareType::relative));

  // Express the expected bias acceleration result in the end-effector frame_E.
  const RotationMatrix<double> R_WE =
      frame_E.CalcRotationMatrixInWorld(*context_);
  const RotationMatrix<double> R_EW = R_WE.inverse();
  const Matrix3X<double> avBias_WEi_E_expected = R_EW * avBias_WEi_W_expected;

  // Form Ei's bias translational acceleration in world frame, expressed in E
  // and ensure it is nearly identical to the expected results.
  const Matrix3X<double> avBias_WEi_E =
      plant_->CalcBiasTranslationalAcceleration(
          *context_, JacobianWrtVariable::kV, frame_E, p_EEi, frame_W, frame_E);

  EXPECT_TRUE(CompareMatrices(avBias_WEi_E, avBias_WEi_E_expected, kTolerance,
                              MatrixCompareType::relative));

  // Ensure CalcBiasTranslationalAcceleration() works when it is passed a single
  // generic position vector, which should returns a single bias acceleration.
  const auto p_EEp = p_EEi.col(0);
  const Vector3<double> avBias_WEp_E =
      plant_->CalcBiasTranslationalAcceleration(
          *context_, JacobianWrtVariable::kV, frame_E, p_EEp, frame_W, frame_E);
  EXPECT_TRUE(CompareMatrices(avBias_WEp_E, avBias_WEi_E_expected.col(0),
                              kTolerance, MatrixCompareType::relative));

  // Ensure CalcBiasTranslationalAcceleration() works when it is passed a
  // MatrixX<double> instead of a Matrix3X<double> of position vectors.
  const MatrixX<double> p_EEj = p_EEi;
  const MatrixX<double> avBias_WEj_E =
      plant_->CalcBiasTranslationalAcceleration(
          *context_, JacobianWrtVariable::kV, frame_E, p_EEi, frame_W, frame_E);
  EXPECT_TRUE(CompareMatrices(avBias_WEj_E, avBias_WEi_E_expected, kTolerance,
                              MatrixCompareType::relative));

  // Verify CalcBiasTranslationalAcceleration() throws an exception if
  // with_respect_to is JacobianWrtVariable::kQDot.
  // TODO(Mitiguy) Remove this test when CalcBiasTranslationalAcceleration() is
  //  improved to handle JacobianWrtVariable::kQDot.
  EXPECT_THROW(plant_->CalcBiasTranslationalAcceleration(
                   *context_, JacobianWrtVariable::kQDot, frame_E, p_EEi,
                   frame_W, frame_W),
               std::exception);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
