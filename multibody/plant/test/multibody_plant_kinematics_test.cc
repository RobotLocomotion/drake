/// @file
/// This file contains tests for kinematics methods in the MultibodyPlant class.
/// There are similar tests in frame_kinematics_test.cc which test
/// kinematics methods in the Frame class.
#include <limits>
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/test_utilities/spatial_derivative.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {

namespace multibody {

namespace {

using Eigen::MatrixXd;
using Eigen::Vector3d;

// Fixture for a two degree-of-freedom pendulum having two links A and B.
// Link A is connected to world (frame W) with a z-axis pin joint (PinJoint1).
// Link B is connected to link A with another z-axis pin joint (PinJoint2).
// Hence links A and B only move in the world's x-y plane (perpendicular to Wz).
// The long axis of link A is parallel to A's unit vector Ax and
// the long axis of link B is parallel to B's unit vector Bx.
// PinJoint1 connects point Wo (world frame W's origin) to link A.
// PinJoint2 connects point Fo (frame F's origin) and Mo (frame M's origin)
// where frame F is fixed/welded to link A and frame M is fixed to link B.
// In the baseline configuration, the origin points Wo, Ao, Fo, Mo, Bo, are
// sequential along the links (they form a line parallel to Wx = Ax = Bx).
// Note: The applied forces (such as gravity) on this system are irrelevant as
// the plan for this text fixture is limited to testing kinematics for an
// instantaneous given state.  The mass/inertia properties of this plant are
// irrelevant and there are no actuators, sensors, controller, or ports.
class TwoDOFPlanarPendulumTest : public ::testing::Test {
 public:
  // Setup the MBP.
  void SetUp() override {
    // Set a spatial inertia for each link.
    const double link_mass = 5.0;  // kg
    const SpatialInertia<double> link_central_inertia =
        SpatialInertia<double>::ThinRodWithMass(link_mass, link_length_,
                                                Vector3<double>::UnitX());

    // Add the two links to the MultibodyPlant.
    bodyA_ = &plant_.AddRigidBody("BodyA", link_central_inertia);
    bodyB_ = &plant_.AddRigidBody("BodyB", link_central_inertia);

    // Create a revolute joint that connects point Wo of the world frame to a
    // unnamed point of link A.  The revolute joint is a distance link_length/2
    // from link A's centroid point Ao (which is also A's center of mass Acm).
    const Vector3<double> p_AoWo_A(-0.5 * link_length_, 0.0, 0.0);
    joint1_ = &plant_.AddJoint<RevoluteJoint>(
        "PinJoint1", plant_.world_body(), std::nullopt, *bodyA_,
        math::RigidTransformd(p_AoWo_A), Vector3<double>::UnitZ());

    // Create a revolute joint that connects point Fo (frame F's origin) to
    // point Mo (frame M's origin), where frame F is fixed/welded to the
    // distal end of link A (Fo is a distance of link_length/2 from Ao) and
    // frame M is fixed/welded to link B.  Mo is a distance of link_length/2
    // from link B's centroid point Bo (which is also B's center of mass Bcm).
    const Vector3<double> p_AoFo_A(0.5 * link_length_, 0.0, 0.0);
    const Vector3<double> p_BoMo_B(-0.5 * link_length_, 0.0, 0.0);
    joint2_ = &plant_.AddJoint<RevoluteJoint>(
        "PinJoint2", *bodyA_, math::RigidTransformd(p_AoFo_A), *bodyB_,
        math::RigidTransformd(p_BoMo_B), Vector3<double>::UnitZ());

    // Finalize the plant and create a context to store this plant's state.
    plant_.Finalize();
    context_ = plant_.CreateDefaultContext();

    // Set joints angle and rates to default values.
    joint1_->set_angle(context_.get(), qA_);
    joint2_->set_angle(context_.get(), qB_);
    joint1_->set_angular_rate(context_.get(), wAz_);
    joint2_->set_angular_rate(context_.get(), wBz_);
  }

  // For a generic point Ap of link A, this helper function implements a by-hand
  // calculation of Ap's spatial acceleration in world W, expressed in link A.
  // param[in] wAzdot Az measure of link A's angular acceleration in world W.
  // param[in] p_WoAp_A point Ap's position from Wo (fixed in world W).
  SpatialAcceleration<double> CalcApSpatialAccelerationInWExpressedInA(
      const double wAzdot, const Vector3<double>& p_WoAp_A) {
    const Vector3<double> w_WA_A(0, 0, wAz_);        //     w_WA_A = wAz Az
    const Vector3<double> alpha_WA_A(0, 0, wAzdot);  // alpha_WA_A = wAzdot Az
    // a_WAp_A = alpha_WA_A x p_WoAp_A + w_WA_A x (w_WA_A x p_WoAp_A).
    const Vector3<double> a_WAp_A =
        alpha_WA_A.cross(p_WoAp_A) + w_WA_A.cross(w_WA_A.cross(p_WoAp_A));
    return SpatialAcceleration<double>(alpha_WA_A, a_WAp_A);
  }

  // For a generic point Bp of link B, this helper function implements a by-hand
  // calculation of Bp's spatial acceleration in link A, expressed in link A.
  // param[in] wBzdot Az measure of link B's angular acceleration in link A.
  // param[in] p_MoBp_B point Bp's position from point Mo (B's joint point).
  SpatialAcceleration<double> CalcBpSpatialAccelerationInAExpressedInA(
      const double wBzdot, const Vector3<double>& p_MoBp_B) {
    const Vector3<double> w_AB_A(0, 0, wBz_);        //     w_AB_A = wBz Az
    const Vector3<double> alpha_AB_A(0, 0, wBzdot);  // alpha_AB_A = wBzdot Az

    // Express position vector argument p_MoBp_B in terms of link A's frame.
    const math::RotationMatrix<double> R_AB =
        math::RotationMatrix<double>::MakeZRotation(qB_);
    const Vector3<double> p_MoBp_A = R_AB * p_MoBp_B;

    // a_ABp_A = alpha_AB_A x p_MoBp_A + w_AB_A x (w_AB_A x p_MoBp_A).
    const Vector3<double> a_ABp_A =
        alpha_AB_A.cross(p_MoBp_A) + w_AB_A.cross(w_AB_A.cross(p_MoBp_A));
    return SpatialAcceleration<double>(alpha_AB_A, a_ABp_A);
  }

  // For a generic point Bp of link B, this helper function implements a by-hand
  // calculation of Bp's spatial acceleration in world W, expressed in link A.
  // param[in] wBzdot Az measure of link B's angular acceleration in link A.
  // param[in] p_MoBp_B point Bp's position from point Mo (B's joint point).
  SpatialAcceleration<double> CalcBpSpatialAccelerationInWExpressedInA(
      const VectorX<double>& vdot, const Vector3<double>& p_MoBp_B) {
    const Vector3<double> p_WoMo_A(link_length_, 0, 0);
    const SpatialAcceleration<double> A_WMo_A =
        CalcApSpatialAccelerationInWExpressedInA(vdot[0], p_WoMo_A);
    const Vector3<double>& a_WMo_A = A_WMo_A.translational();

    // alpha_WB_A = DtW(w_WB_A) = (ẇAz + ẇBz) Az
    const Vector3<double> w_WB_A(0, 0, wAz_ + wBz_);  // w_WB_A = (wAz + wBz) Az
    const Vector3<double> alpha_WB_A(0, 0, vdot[0] + vdot[1]);

    // Express position vector argument p_MoBp_B in terms of link A's frame.
    const math::RotationMatrix<double> R_AB =
        math::RotationMatrix<double>::MakeZRotation(qB_);
    const Vector3<double> p_MoBp_A = R_AB * p_MoBp_B;

    // a_WBp_A = a_WMo_A + alpha_WB_A x p_MoBp_A + w_WB_A x (w_WB_A x p_MoBp_A).
    const Vector3<double> a_WBp_A = a_WMo_A + alpha_WB_A.cross(p_MoBp_A) +
                                    w_WB_A.cross(w_WB_A.cross(p_MoBp_A));
    return SpatialAcceleration<double>(alpha_WB_A, a_WBp_A);
  }

 protected:
  // Since the maximum absolute value of acceleration in this test is
  // approximately ω² * (2 * link_length) ≈ 72 , we test that the errors in
  // acceleration calculations are less than 7 bits (2^7 = 128).
  const double kTolerance = 128 * std::numeric_limits<double>::epsilon();
  const double link_length_ = 4.0;  // meters
  const double qA_ = M_PI / 6.0;    // Link A's angle in world W, about Az = Nz.
  const double qB_ = M_PI / 12.0;   // Link B's angle in link A,  about Bz = Az.
  const double wAz_ = 3.0;          // rad/sec
  const double wBz_ = 2.0;          // rad/sec
  MultibodyPlant<double> plant_{0.0};
  std::unique_ptr<systems::Context<double>> context_;
  const RigidBody<double>* bodyA_{nullptr};
  const RigidBody<double>* bodyB_{nullptr};
  const RevoluteJoint<double>* joint1_{nullptr};
  const RevoluteJoint<double>* joint2_{nullptr};
};

// The point of this test is to
// a. Verify the utility method
//    test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
// b. Show how that utility method can be used to test
//    MultibodyPlant::CalcBiasSpatialAcceleration().
TEST_F(TwoDOFPlanarPendulumTest, CalcBiasSpatialAcceleration) {
  // To calculate bias spatial accelerations, set vdot = 0 [ẇAz = 0, ẇBz = 0].
  const Eigen::VectorXd vdot = Eigen::Vector2d(0.0, 0.0);

  // Point Ap is the point of A located at the revolute joint connecting link A
  // and link B.  Calculate Ap's bias spatial acceleration in world W.
  const Frame<double>& frame_A = bodyA_->body_frame();
  const Frame<double>& frame_W = plant_.world_frame();
  const Vector3<double> p_AoAp_A(0.5 * link_length_, 0.0, 0.0);
  const SpatialAcceleration<double> ABias_WAp_A =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_A, p_AoAp_A, frame_W, frame_A);

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match by-hand result in CalcApSpatialAccelerationInWExpressedInA().
  // aBias_WAp_A = -L wAz_² Ax.
  const Vector3<double> p_WoAp_A(link_length_, 0.0, 0.0);
  const SpatialAcceleration<double> ABias_WAp_A_expected =
      CalcApSpatialAccelerationInWExpressedInA(0.0, p_WoAp_A);
  EXPECT_TRUE(CompareMatrices(ABias_WAp_A_expected.get_coeffs(),
                              ABias_WAp_A.get_coeffs(), kTolerance));

  // Verify MultibodyPlant::CalcBiasSpatialAcceleration() results match those
  // of test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
  const SpatialAcceleration<double> ABias_WAp_A_alternate =
      plant_.CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                         frame_A, p_AoAp_A, frame_W, frame_A);
  EXPECT_TRUE(CompareMatrices(ABias_WAp_A_alternate.get_coeffs(),
                              ABias_WAp_A.get_coeffs(), kTolerance));

  // Point Bp is the point of B located at the most distal end of link B.
  // Calculate Bp's bias spatial acceleration in world W, expressed in A.
  const Frame<double>& frame_B = bodyB_->body_frame();
  const Vector3<double> p_BoBp_B(0.5 * link_length_, 0.0, 0.0);
  const SpatialAcceleration<double> ABias_WBp_A =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_B, p_BoBp_B, frame_W, frame_A);

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match by-hand result in CalcBpSpatialAccelerationInWExpressedInA().
  // aBias_WBp_A = -L wAz_² Ax - L (wAz_ + wBz_)² Bx
  Vector3<double> p_MoBp_B(link_length_, 0.0, 0.0);
  const SpatialAcceleration<double> ABias_WBp_A_expected =
      CalcBpSpatialAccelerationInWExpressedInA(vdot, p_MoBp_B);
  EXPECT_TRUE(CompareMatrices(ABias_WBp_A_expected.get_coeffs(),
                              ABias_WBp_A.get_coeffs(), kTolerance));

  // Verify MultibodyPlant::CalcBiasSpatialAcceleration() results match those
  // of test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
  const SpatialAcceleration<double> ABias_WBp_A_alternate =
      plant_.CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                         frame_B, p_BoBp_B, frame_W, frame_A);
  EXPECT_TRUE(CompareMatrices(ABias_WBp_A_alternate.get_coeffs(),
                              ABias_WBp_A.get_coeffs(), kTolerance));

  // Point Ap is the point of A located at the revolute joint connecting link A
  // and link B.  Calculate Ap's bias spatial acceleration in frame A.
  const SpatialAcceleration<double> ABias_AAp_W =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_A, p_AoAp_A, frame_A, frame_W);

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match simple by-hand result: aBias_AAp_W = Vector3d::Zero().
  EXPECT_TRUE(CompareMatrices(ABias_AAp_W.get_coeffs(),
                              Vector6d::Zero(), kTolerance));

  // Verify MultibodyPlant::CalcBiasSpatialAcceleration() results match those
  // of test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
  const SpatialAcceleration<double> ABias_AAp_W_alternate =
      plant_.CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                         frame_A, p_AoAp_A, frame_A, frame_W);
  EXPECT_TRUE(CompareMatrices(ABias_AAp_W_alternate.get_coeffs(),
                              ABias_AAp_W.get_coeffs(), kTolerance));

  // Point Bp is the point of B located at the most distal end of link B.
  // Calculate Bp's bias spatial acceleration in frame A, expressed in A.
  const SpatialAcceleration<double> ABias_ABp_A =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_B, p_BoBp_B, frame_A, frame_A);

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match by-hand result in CalcBpSpatialAccelerationInAExpressedInA().
  // aBias_ABp_A = -L wBz_² Ax
  const SpatialAcceleration<double> ABias_ABp_A_expected =
      CalcBpSpatialAccelerationInAExpressedInA(vdot[1], p_MoBp_B);
  EXPECT_TRUE(CompareMatrices(ABias_ABp_A_expected.get_coeffs(),
                              ABias_ABp_A.get_coeffs(), kTolerance));

  // Verify MultibodyPlant::CalcBiasSpatialAcceleration() results match those
  // of test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
  const SpatialAcceleration<double> ABias_ABp_A_alternate =
      plant_.CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                         frame_B, p_BoBp_B, frame_A, frame_A);
  EXPECT_TRUE(CompareMatrices(ABias_ABp_A_alternate.get_coeffs(),
                              ABias_ABp_A.get_coeffs(), kTolerance));

  // Point Bq is the point of B located at from Bo as shown below.
  // Calculate Bq's bias spatial acceleration in link A, expressed in frame B.
  const double x = 0.1, y = -0.2, z = 0.3;
  const Vector3<double> p_BoBq_B(x, y, z);
  const SpatialAcceleration<double> ABias_ABq_B =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_B, p_BoBq_B, frame_A, frame_B);

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match by-hand result in CalcBpSpatialAccelerationInAExpressedInA().
  // aBias_ABq_B = -(L/2 + x) wBz_² Bx - y wBz_² By.
  p_MoBp_B = Vector3<double>(0.5*link_length_, 0.0, 0.0) + p_BoBq_B;
  const SpatialAcceleration<double> ABias_ABq_A_expected =
      CalcBpSpatialAccelerationInAExpressedInA(vdot[1], p_MoBp_B);
  const math::RotationMatrix<double> R_BA =
      math::RotationMatrix<double>::MakeZRotation(-qB_);
  const SpatialAcceleration<double> ABias_ABq_B_expected =
      R_BA * ABias_ABq_A_expected;
  EXPECT_TRUE(CompareMatrices(ABias_ABq_B_expected.get_coeffs(),
                              ABias_ABq_B.get_coeffs(), kTolerance));

  // Verify via hand-result: aBias_ABq_B = -0.5 (L+2*x) wBz_² Bx - y wBz_² By.
  const double wB_squared = wBz_ * wBz_;
  const Vector3<double> aBias_ABq_B_expected(
      -(0.5*link_length_ + x) * wB_squared, -y * wB_squared, 0);
  EXPECT_TRUE(CompareMatrices(ABias_ABq_B.translational(), aBias_ABq_B_expected,
                              kTolerance));

  // Verify MultibodyPlant::CalcBiasSpatialAcceleration() results match those
  // of test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
  const SpatialAcceleration<double> ABias_ABq_B_alternate =
      plant_.CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                         frame_B, p_BoBq_B, frame_A, frame_B);
  EXPECT_TRUE(CompareMatrices(ABias_ABq_B_alternate.get_coeffs(),
                              ABias_ABq_B.get_coeffs(), kTolerance));
}

// The point of this test is to
// a. Verify the utility method
//    test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
// b. Show how that utility method can be used to test
//    MultibodyPlant::CalcSpatialAcceleration().
TEST_F(TwoDOFPlanarPendulumTest, CalcSpatialAcceleration) {
  // To calculate spatial accelerations, set vdot to arbitrary values.
  const double wAzdot = 10, wBzdot = 20;  // Link angular accelerations.
  const Eigen::VectorXd vdot = Eigen::Vector2d(wAzdot, wBzdot);

  // Shortcuts to various frames.
  const Frame<double>& frame_W = plant_.world_frame();
  const Frame<double>& frame_A = bodyA_->body_frame();
  const Frame<double>& frame_B = bodyB_->body_frame();

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // produce zero vector for frame A's spatial acceleration in A: a_AAo_A = 0.
  const Vector3<double> p_AoAo_A = Vector3<double>::Zero();
  const SpatialAcceleration<double> A_AAo_A =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_A, p_AoAo_A, frame_A, frame_A);
  EXPECT_TRUE(CompareMatrices(A_AAo_A.get_coeffs(), Vector6<double>::Zero(),
                              kTolerance));

  // Point Ao is the point of A located at its centroid (so Ao is Acm).
  // Calculate Ao's spatial acceleration in world W, expressed in link frame A.
  const SpatialAcceleration<double> A_WAo_A =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_A, p_AoAo_A, frame_W, frame_A);

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match by-hand result in CalcApSpatialAccelerationInWExpressedInA().
  // a_AAo_A = -0.5 L wAz_² Ax + 0.5 L ẇAz Ay
  const Vector3<double> p_WoAo_W(0.5 * link_length_, 0, 0);
  const SpatialAcceleration<double> A_WAo_A_expected =
      CalcApSpatialAccelerationInWExpressedInA(wAzdot, p_WoAo_W);
  EXPECT_TRUE(CompareMatrices(A_WAo_A.get_coeffs(),
                              A_WAo_A_expected.get_coeffs(), kTolerance));

  // TODO(Mitiguy) Verify frame_A.CalcSpatialAcceleration() results match those
  //  of test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
  //  Use code something like the following:
  //  const SpatialAcceleration<double> A_WAo_A_alternate =
  //   plant_.CalcSpatialAcceleration(*context_, frame_A, p_AoAp_A,
  //       frame_W, frame_A);
  //  EXPECT_TRUE(CompareMatrices(A_WAo_A_alternate.get_coeffs(),
  //                              A_WAo_A.get_coeffs(), kTolerance));

  // Another way to check the results is setting vdot for all the joints, doing
  // inverse dynamics, and calculating the spatial acceleration of all bodies.
  // Herein, A_WallBo_W is an array of the spatial acceleration of all bodies
  // measured in the world frame W, and expressed in W.
  std::vector<SpatialAcceleration<double>> A_WallBo_W(plant_.num_bodies());
  plant_.CalcSpatialAccelerationsFromVdot(*context_, vdot, &A_WallBo_W);

  // The 0th, 1st, and 2nd elements of A_WallBo_W correspond to the world body,
  // bodyA_, and bodyB_, respectively.
  const SpatialAcceleration<double> A_WAo_W = A_WallBo_W.at(1);
  const SpatialAcceleration<double> A_WBo_W = A_WallBo_W.at(2);

  // Verify MultibodyPlant::CalcSpatialAccelerationsFromVdot() results match
  // with test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()..
  const math::RotationMatrix<double> R_WA =
      frame_A.CalcRotationMatrixInWorld(*context_);
  EXPECT_TRUE(CompareMatrices(A_WAo_W.get_coeffs(),
                              (R_WA * A_WAo_A).get_coeffs(), kTolerance));

#if 0
  // Compare A_WAo_W with result from frame_A.CalcSpatialAccelerationInWorld().
  const SpatialAcceleration<double> A_WAo_W_alternate =
      frame_A.CalcSpatialAccelerationInWorld(*context_);
  EXPECT_TRUE(CompareMatrices(A_WAo_W_alternate.get_coeffs(),
                              A_WAo_W.get_coeffs(), kTolerance));
#endif

#if 0
  const SpatialAcceleration<double> A_WAo_A_alternate =
      frame_A.CalcSpatialAcceleration(*context_, frame_W, frame_A);
  EXPECT_TRUE(CompareMatrices(A_WAo_A_alternate.get_coeffs(),
      A_WAo_A.get_coeffs(), kTolerance));
#endif

  // Point Bo is the point of B located at its centroid (so Bo is Bcm).
  // Calculate Bo's spatial acceleration in world W, expressed in link frame A.
  const Vector3<double> p_BoBo_B = Vector3<double>::Zero();
  const SpatialAcceleration<double> A_WBo_A =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_B, p_BoBo_B, frame_W, frame_A);

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match by-hand result in CalcBpSpatialAccelerationInWExpressedInA().
  Vector3<double> p_MoBo_B(0.5 * link_length_, 0, 0);
  const SpatialAcceleration<double> A_WBo_A_expected =
      CalcBpSpatialAccelerationInWExpressedInA(vdot, p_MoBo_B);
  EXPECT_TRUE(CompareMatrices(A_WBo_A.get_coeffs(),
                              A_WBo_A_expected.get_coeffs(), kTolerance));

  // Verify MultibodyPlant::CalcSpatialAccelerationsFromVdot() results match
  // with test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
  EXPECT_TRUE(CompareMatrices(A_WBo_W.get_coeffs(),
                              (R_WA * A_WBo_A).get_coeffs(), kTolerance));

  // Point Ap is the point of A located at the revolute joint connecting link A
  // and link B (i.e., frame_Ap is fixed to A that is joint2's parent frame).
  // Calculate frame_Ap's spatial acceleration in frame W, expressed in A.
  const Vector3<double> p_ApAp_Ap = Vector3<double>::Zero();
  const Frame<double>& frame_Ap = joint2_->frame_on_parent();
  const SpatialAcceleration<double> A_WAp_A =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_Ap, p_ApAp_Ap, frame_W, frame_A);

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match by-hand result in CalcApSpatialAccelerationInWExpressedInA().
  const Vector3<double> p_WoAp_W(link_length_, 0, 0);
  const SpatialAcceleration<double> A_WAp_A_expected =
      CalcApSpatialAccelerationInWExpressedInA(wAzdot, p_WoAp_W);
  EXPECT_TRUE(CompareMatrices(A_WAp_A.get_coeffs(),
                              A_WAp_A_expected.get_coeffs(), kTolerance));

  // Point Bp is the point of B located at the most distal end of link B.
  // Calculate Bp's spatial acceleration in frame W, expressed in A.
  Vector3<double> p_BoBp_B(0.5 * link_length_, 0, 0);
  const SpatialAcceleration<double> A_WBp_A =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_B, p_BoBp_B, frame_W, frame_A);

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match by-hand result in CalcBpSpatialAccelerationInWExpressedInA().
  const Vector3<double> p_MoBp_B(link_length_, 0, 0);
  const SpatialAcceleration<double> A_WBp_A_expected =
      CalcBpSpatialAccelerationInWExpressedInA(vdot, p_MoBp_B);
  EXPECT_TRUE(CompareMatrices(A_WBp_A.get_coeffs(),
                              A_WBp_A_expected.get_coeffs(), kTolerance));

  // Calculate frame_Bp's spatial acceleration in frame A, expressed in A.
  const SpatialAcceleration<double> A_ABp_A =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_B, p_BoBp_B, frame_A, frame_A);

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match by-hand result in CalcBpSpatialAccelerationInAExpressedInA().
  const SpatialAcceleration<double> A_ABp_A_expected =
      CalcBpSpatialAccelerationInAExpressedInA(wBzdot, p_MoBp_B);
  EXPECT_TRUE(CompareMatrices(A_ABp_A.get_coeffs(),
                              A_ABp_A_expected.get_coeffs(), kTolerance));

  // Verify the previous results with the following by-hand analytical results.
  // DtA(w_AB_A)  = Dt(wBz) Az
  //              = alphaB Az
  // DtA(v_ABp_A) = DtB(v_ABp_A) + w_AB_A x v_ABp_A
  //              = L alphaB By + wBz Bz x L wBz By.
  //              = L alphaB By - L wBz² Bx
  // Reminder: Ax = Bx, Ay = By, Az = Bz when qB = 0°.  L = link_length_.
  const double alphaB = wBzdot;
  const double wB_squared = wBz_ * wBz_;
  const Vector3<double> alpha_AB_A_expected(0, 0, alphaB);
  const Vector3<double> a_ABp_B_expected(-link_length_ * wB_squared,
                                         link_length_ * alphaB, 0);
  const math::RotationMatrix<double> R_AB =
      math::RotationMatrix<double>::MakeZRotation(qB_);
  const Vector3<double> a_ABp_A_expected = R_AB * a_ABp_B_expected;
  EXPECT_TRUE(
      CompareMatrices(A_ABp_A.rotational(), alpha_AB_A_expected, kTolerance));
  EXPECT_TRUE(
      CompareMatrices(A_ABp_A.translational(), a_ABp_A_expected, kTolerance));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
