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

// Fixture for a two degree-of-freedom pendulum having two thin links A and B.
// Link A is connected to world (frame W) with a z-axis pin joint (PinJoint1).
// Link B is connected to link A with another z-axis pin joint (PinJoint2).
// Hence links A and B only move in the world's x-y plane (perpendicular to Wz).
// The long axis of link A is parallel to A's unit vector Ax and
// the long axis of link B is parallel to B's unit vector Bx.
// PinJoint1 connects point Wo (world W's origin) to link A at point Am of A.
// PinJoint2 connects point Bm of link B to point Af of link A.
// In the baseline configuration, points Wo, Am, Ao, Af, Bm, Bo, are sequential
// along the links (they form a line parallel to Wx = Ax = Bx).
// Note: The applied forces (such as gravity) on this system are irrelevant as
// this text fixture is currently only used to test kinematics for an
// instantaneous given state.  The mass/inertia properties of this plant are
// irrelevant and there are no actuators, sensors, controller, or ports.
class TwoDOFPlanarPendulumTest : public ::testing::Test {
 public:
  // Setup the multibody plant.
  void SetUp() override {
    // Set a spatial inertia for each link.
    const double link_mass = 5.0;  // kg
    const SpatialInertia<double> link_central_inertia =
        SpatialInertia<double>::ThinRodWithMass(link_mass, link_length_,
                                                Vector3<double>::UnitX());

    // Add the two links to the MultibodyPlant.
    bodyA_ = &plant_.AddRigidBody("BodyA", link_central_inertia);
    bodyB_ = &plant_.AddRigidBody("BodyB", link_central_inertia);

    // Make a pin joint that connects point Wo of world W to point Am of link A.
    joint1_ = &plant_.AddJoint<RevoluteJoint>(
        "PinJoint1", plant_.world_body(), std::nullopt, *bodyA_,
        math::RigidTransformd(p_AoAm_A_), Vector3<double>::UnitZ());

    // Make a pin joint that connects point Af of link A to point Bm of link B.
    joint2_ = &plant_.AddJoint<RevoluteJoint>(
        "PinJoint2", *bodyA_, math::RigidTransformd(p_AoAf_A_), *bodyB_,
        math::RigidTransformd(p_BoBm_B_), Vector3<double>::UnitZ());

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
  // param[in] wAzdot time-derivative of wAz_.
  // param[in] p_AoAp_A point Ap's position from point Ao (link A's centroid).
  SpatialAcceleration<double> CalcApSpatialAccelerationInWExpressedInA(
      const double wAzdot, const Vector3<double>& p_AoAp_A) {
    const Vector3<double> p_AmAp_A = -p_AoAm_A_ + p_AoAp_A;
    const Vector3<double> w_WA_A(0, 0, wAz_);        //     w_WA_A = wAz Az
    const Vector3<double> alpha_WA_A(0, 0, wAzdot);  // alpha_WA_A = ẇAz Az
    // a_WAp_A = alpha_WA_A x p_AmAp_A + w_WA_A x (w_WA_A x p_AmAp_A).
    const Vector3<double> a_WAp_A =
        alpha_WA_A.cross(p_AmAp_A) + w_WA_A.cross(w_WA_A.cross(p_AmAp_A));
    return SpatialAcceleration<double>(alpha_WA_A, a_WAp_A);
  }

  // For a generic point Bp of link B, this helper function implements a by-hand
  // calculation of Bp's spatial acceleration in link A, expressed in link A.
  // param[in] wBzdot time-derivative of wBz_.
  // param[in] p_BoBp_B point Bp's position from point Bo (link B's centroid).
  SpatialAcceleration<double> CalcBpSpatialAccelerationInAExpressedInA(
      const double wBzdot, const Vector3<double>& p_BoBp_B) {
    const Vector3<double> w_AB_A(0, 0, wBz_);        //     w_AB_A = wBz Az
    const Vector3<double> alpha_AB_A(0, 0, wBzdot);  // alpha_AB_A = ẇBz Az

    // Express position vector p_BmBp_B in terms of link A's frame.
    const Vector3<double> p_BmBp_B = -p_BoBm_B_ + p_BoBp_B;
    const math::RotationMatrix<double> R_AB =
        math::RotationMatrix<double>::MakeZRotation(qB_);
    const Vector3<double> p_BmBp_A = R_AB * p_BmBp_B;

    // a_ABp_A = alpha_AB_A x p_BmBp_A + w_AB_A x (w_AB_A x p_BmBp_A).
    const Vector3<double> a_ABp_A =
        alpha_AB_A.cross(p_BmBp_A) + w_AB_A.cross(w_AB_A.cross(p_BmBp_A));
    return SpatialAcceleration<double>(alpha_AB_A, a_ABp_A);
  }

  // For a generic point Bp of link B, this helper function implements a by-hand
  // calculation of Bp's spatial acceleration in world W, expressed in link A.
  // param[in] vdot contains [ẇAz, ẇBz], the time-derivatives of wAz_ and wBz_.
  // param[in] p_BoBp_B point Bp's position from point Bo (link B's centroid).
  SpatialAcceleration<double> CalcBpSpatialAccelerationInWExpressedInA(
      const VectorX<double>& vdot, const Vector3<double>& p_BoBp_B) {
    const SpatialAcceleration<double> A_WAf_A =
        CalcApSpatialAccelerationInWExpressedInA(vdot[0], p_AoAf_A_);
    const Vector3<double>& a_WAf_A = A_WAf_A.translational();

    // w_WB_A = (wAz + wBz) Az   Reminder: Bz = Az = Wz (planar mechanism).
    const Vector3<double> w_WB_A(0, 0, wAz_ + wBz_);

    // alpha_WB_A = DtW(w_WB_A) = (ẇAz + ẇBz) Az
    const Vector3<double> alpha_WB_A(0, 0, vdot[0] + vdot[1]);

    // Express position vector p_BmBp_B in terms of link A's frame.
    const Vector3<double> p_BmBp_B = -p_BoBm_B_ + p_BoBp_B;
    const math::RotationMatrix<double> R_AB =
        math::RotationMatrix<double>::MakeZRotation(qB_);
    const Vector3<double> p_BmBp_A = R_AB * p_BmBp_B;

    // a_WBp_A = a_WAf_A + alpha_WB_A x p_BmBp_A + w_WB_A x (w_WB_A x p_BmBp_A).
    const Vector3<double> a_WBp_A = a_WAf_A + alpha_WB_A.cross(p_BmBp_A) +
                                    w_WB_A.cross(w_WB_A.cross(p_BmBp_A));
    return SpatialAcceleration<double>(alpha_WB_A, a_WBp_A);
  }

  // Calculate and return [ẇAz, ẇBz], the time derivative of [wAz, wBz].
  Vector2<double> CalcVdotForNoAppliedForces() {
    // The calculation below was generated by MotionGenesis.
    const double mLsq = link_mass_ * link_length_ * link_length_;
    const double Izz = mLsq / 12.0;
    const double z1 = 2.0 * Izz + 1.5 * mLsq + mLsq * std::cos(qB_);
    const double z2 = Izz + 0.25 * mLsq * (1 + 2 * std::cos(qB_));
    const double z3 = mLsq * sin(qB_);
    const double z4 = Izz + 0.25 * mLsq;
    const double z5 = wBz_ * (wBz_ + 2 * wAz_);
    const double z2sq = z2 * z2;
    const double wAzsq = wAz_ * wAz_;
    const double denom = z4 * z1 - z2sq;
    const double wADt = 0.5 * z3 * (z2 * wAzsq + z4 * z5) / denom;
    const double wBDt = -0.5 * z3 * (z1 * wAzsq + z2 * z5) / denom;
    return Vector2<double>(wADt, wBDt);  // Returns [ẇAz, ẇBz]
  }

 protected:
  // Since the maximum absolute value of acceleration in this test is
  // approximately ω² * (2 * link_length) ≈ 72 , we test that the errors in
  // acceleration calculations are less than 7 bits (2^7 = 128).
  const double kTolerance = 128 * std::numeric_limits<double>::epsilon();
  const double link_length_ = 4.0;  // meters
  const double link_mass_ = 5.0;    // kilograms
  const double qA_ = M_PI / 6.0;    // Link A's angle in world W, about Az = Wz.
  const double qB_ = M_PI / 12.0;   // Link B's angle in link A,  about Bz = Az.
  const double wAz_ = 3.0;          // rad/sec
  const double wBz_ = 2.0;          // rad/sec
  MultibodyPlant<double> plant_{0.0};
  std::unique_ptr<systems::Context<double>> context_;
  const RigidBody<double>* bodyA_{nullptr};
  const RigidBody<double>* bodyB_{nullptr};
  const RevoluteJoint<double>* joint1_{nullptr};
  const RevoluteJoint<double>* joint2_{nullptr};

  // Store the position vector from Ao (link A's centroid) to Am (the point of
  // link A located at the revolute joint that connects link A to world W).
  const Vector3<double> p_AoAm_A_{-0.5 * link_length_, 0.0, 0.0};

  // Store the position vector from Ao (link A's centroid) to Af (the point of
  // link A located at the revolute joint that connects link A to link B).
  const Vector3<double> p_AoAf_A_{0.5 * link_length_, 0.0, 0.0};

  // Store the position vector from Bo (link B's centroid) to Bm (the point of
  // link B located at the revolute joint that connects link B to link A).
  const Vector3<double> p_BoBm_B_{-0.5 * link_length_, 0.0, 0.0};
};

// The point of this test is to
// a. Verify the utility method
//    test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
// b. Show how that utility method can be used to test
//    MultibodyPlant::CalcBiasSpatialAcceleration().
TEST_F(TwoDOFPlanarPendulumTest, CalcBiasSpatialAcceleration) {
  // To calculate bias spatial accelerations, set vdot = [ẇAz, ẇBz] = [0, 0].
  const Eigen::VectorXd vdot = Eigen::Vector2d(0.0, 0.0);

  // Point Ap is an arbitrary point of link A.
  // Calculate Ap's bias spatial acceleration in world W, expressed in frame A.
  const Frame<double>& frame_A = bodyA_->body_frame();
  const Frame<double>& frame_W = plant_.world_frame();
  const Vector3<double> p_AoAp_A(0.1, -0.2, 0.3);
  const SpatialAcceleration<double> ABias_WAp_A =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_A, p_AoAp_A, frame_W, frame_A);

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match by-hand result in CalcApSpatialAccelerationInWExpressedInA().
  const SpatialAcceleration<double> ABias_WAp_A_expected =
      CalcApSpatialAccelerationInWExpressedInA(0.0, p_AoAp_A);
  EXPECT_TRUE(CompareMatrices(ABias_WAp_A_expected.get_coeffs(),
                              ABias_WAp_A.get_coeffs(), kTolerance));

  // Verify MultibodyPlant::CalcBiasSpatialAcceleration() results match those
  // of test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
  const SpatialAcceleration<double> ABias_WAp_A_alternate =
      plant_.CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                         frame_A, p_AoAp_A, frame_W, frame_A);
  EXPECT_TRUE(CompareMatrices(ABias_WAp_A_alternate.get_coeffs(),
                              ABias_WAp_A.get_coeffs(), kTolerance));

  // Point Bp is an arbitrary point of link B.
  // Calculate Bp's bias spatial acceleration in world W, expressed in frame A.
  const Frame<double>& frame_B = bodyB_->body_frame();
  const Vector3<double> p_BoBp_B(0.4, -0.5, 0.6);
  const SpatialAcceleration<double> ABias_WBp_A =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_B, p_BoBp_B, frame_W, frame_A);

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match by-hand result in CalcBpSpatialAccelerationInWExpressedInA().
  const SpatialAcceleration<double> ABias_WBp_A_expected =
      CalcBpSpatialAccelerationInWExpressedInA(vdot, p_BoBp_B);
  EXPECT_TRUE(CompareMatrices(ABias_WBp_A_expected.get_coeffs(),
                              ABias_WBp_A.get_coeffs(), kTolerance));

  // Verify MultibodyPlant::CalcBiasSpatialAcceleration() results match those
  // of test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
  const SpatialAcceleration<double> ABias_WBp_A_alternate =
      plant_.CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                         frame_B, p_BoBp_B, frame_W, frame_A);
  EXPECT_TRUE(CompareMatrices(ABias_WBp_A_alternate.get_coeffs(),
                              ABias_WBp_A.get_coeffs(), kTolerance));

  // Point Ap is an arbitrary point of link A.
  // Calculate Ap's bias spatial acceleration in frame A and ensure it is zero.
  const SpatialAcceleration<double> ABias_AAp_W =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_A, p_AoAp_A, frame_A, frame_W);
  EXPECT_TRUE(
      CompareMatrices(ABias_AAp_W.get_coeffs(), Vector6d::Zero(), kTolerance));

  // Verify MultibodyPlant::CalcBiasSpatialAcceleration() results match those
  // of test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
  const SpatialAcceleration<double> ABias_AAp_W_alternate =
      plant_.CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                         frame_A, p_AoAp_A, frame_A, frame_W);
  EXPECT_TRUE(CompareMatrices(ABias_AAp_W_alternate.get_coeffs(),
                              Vector6d::Zero(), kTolerance));

  // Point Bp is an arbitrary point of link B.
  // Calculate Bp's bias spatial acceleration in link A, expressed in frame A.
  const SpatialAcceleration<double> ABias_ABp_A =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_B, p_BoBp_B, frame_A, frame_A);

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match by-hand result in CalcBpSpatialAccelerationInAExpressedInA().
  const SpatialAcceleration<double> ABias_ABp_A_expected =
      CalcBpSpatialAccelerationInAExpressedInA(0.0, p_BoBp_B);
  EXPECT_TRUE(CompareMatrices(ABias_ABp_A_expected.get_coeffs(),
                              ABias_ABp_A.get_coeffs(), kTolerance));

  // Verify MultibodyPlant::CalcBiasSpatialAcceleration() results match those
  // of test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
  const SpatialAcceleration<double> ABias_ABp_A_alternate =
      plant_.CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                         frame_B, p_BoBp_B, frame_A, frame_A);
  EXPECT_TRUE(CompareMatrices(ABias_ABp_A_alternate.get_coeffs(),
                              ABias_ABp_A.get_coeffs(), kTolerance));

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match by-hand result in CalcBpSpatialAccelerationInAExpressedInA().
  const SpatialAcceleration<double> ABias_ABp_B =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_B, p_BoBp_B, frame_A, frame_B);
  const math::RotationMatrix<double> R_BA =
      math::RotationMatrix<double>::MakeZRotation(-qB_);
  const SpatialAcceleration<double> ABias_ABp_B_expected =
      R_BA * ABias_ABp_A_expected;
  EXPECT_TRUE(CompareMatrices(ABias_ABp_B_expected.get_coeffs(),
                              ABias_ABp_B.get_coeffs(), kTolerance));

  // Verify MultibodyPlant::CalcBiasSpatialAcceleration() results match those
  // of test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
  const SpatialAcceleration<double> ABias_ABp_B_alternate =
      plant_.CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                         frame_B, p_BoBp_B, frame_A, frame_B);
  EXPECT_TRUE(CompareMatrices(ABias_ABp_B_alternate.get_coeffs(),
                              ABias_ABp_B.get_coeffs(), kTolerance));
}

// The point of this test is to
// a. Verify the utility method
//    test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
// b. Show how that utility method can be used to test
//    MultibodyPlant::CalcSpatialAcceleration().
TEST_F(TwoDOFPlanarPendulumTest, CalcSpatialAcceleration) {
  // To calculate spatial accelerations, set vdot to arbitrary values.
  const double wAzdot = 10;  // alpha_WA_A = ẇAz Az
  const double wBzdot = 20;  // alpha_AB_A = ẇBz Az
  const Eigen::VectorXd vdot = Eigen::Vector2d(wAzdot, wBzdot);

  // Shortcuts to various frames.
  const Frame<double>& frame_W = plant_.world_frame();
  const Frame<double>& frame_A = bodyA_->body_frame();
  const Frame<double>& frame_B = bodyB_->body_frame();

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // produce zero vector for frame A's spatial acceleration in A: A_AAo_A = 0.
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
  const SpatialAcceleration<double> A_WAo_A_expected =
      CalcApSpatialAccelerationInWExpressedInA(wAzdot, p_AoAo_A);
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
  // Herein, A_WallBody_W is an array of the spatial acceleration of all bodies
  // measured in the world frame W, and expressed in W.
  std::vector<SpatialAcceleration<double>> A_WallBody_W(plant_.num_bodies());
  plant_.CalcSpatialAccelerationsFromVdot(*context_, vdot, &A_WallBody_W);

  // The 0th, 1st, and 2nd elements of A_WallBody_W correspond to the
  // world body, bodyA_, and bodyB_, respectively.
  const SpatialAcceleration<double> A_WAo_W = A_WallBody_W.at(1);
  const SpatialAcceleration<double> A_WBo_W = A_WallBody_W.at(2);

  // Verify MultibodyPlant::CalcSpatialAccelerationsFromVdot() results match
  // with test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
  const math::RotationMatrix<double> R_WA =
      frame_A.CalcRotationMatrixInWorld(*context_);
  EXPECT_TRUE(CompareMatrices(A_WAo_W.get_coeffs(),
                              (R_WA * A_WAo_A).get_coeffs(), kTolerance));

  // Point Bo is the point of B located at its centroid (so Bo is Bcm).
  // Calculate Bo's spatial acceleration in world W, expressed in link frame A.
  const Vector3<double> p_BoBo_B = Vector3<double>::Zero();
  const SpatialAcceleration<double> A_WBo_A =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_B, p_BoBo_B, frame_W, frame_A);

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match by-hand result in CalcBpSpatialAccelerationInWExpressedInA().
  const SpatialAcceleration<double> A_WBo_A_expected =
      CalcBpSpatialAccelerationInWExpressedInA(vdot, p_BoBo_B);
  EXPECT_TRUE(CompareMatrices(A_WBo_A.get_coeffs(),
                              A_WBo_A_expected.get_coeffs(), kTolerance));

  // Verify MultibodyPlant::CalcSpatialAccelerationsFromVdot() results match
  // with test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
  EXPECT_TRUE(CompareMatrices(A_WBo_W.get_coeffs(),
                              (R_WA * A_WBo_A).get_coeffs(), kTolerance));

  // Point Ap is a generic point of link A.
  // Calculate frame_Ap's spatial acceleration in frame W, expressed in A.
  const Vector3<double> p_AoAp_A(0.7, -0.8, 0.9);
  const Frame<double>& frame_Af = joint2_->frame_on_parent();
  const Vector3<double> p_AfAp_A = -p_AoAf_A_ + p_AoAp_A;
  const SpatialAcceleration<double> A_WAp_A =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_Af, p_AfAp_A, frame_W, frame_A);

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match by-hand result in CalcApSpatialAccelerationInWExpressedInA().
  const SpatialAcceleration<double> A_WAp_A_expected =
      CalcApSpatialAccelerationInWExpressedInA(wAzdot, p_AoAp_A);
  EXPECT_TRUE(CompareMatrices(A_WAp_A.get_coeffs(),
                              A_WAp_A_expected.get_coeffs(), kTolerance));

  // Point Bp is an arbitrary point of link B.
  // Calculate Bp's spatial acceleration in world W, expressed in frame A.
  const Vector3<double> p_BoBp_B(0.9, -0.8, 0.7);
  const SpatialAcceleration<double> A_WBp_A =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_B, p_BoBp_B, frame_W, frame_A);

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match by-hand result in CalcBpSpatialAccelerationInWExpressedInA().
  const SpatialAcceleration<double> A_WBp_A_expected =
      CalcBpSpatialAccelerationInWExpressedInA(vdot, p_BoBp_B);
  EXPECT_TRUE(CompareMatrices(A_WBp_A.get_coeffs(),
                              A_WBp_A_expected.get_coeffs(), kTolerance));

  // Calculate Bp's spatial acceleration in frame A, expressed in A.
  const SpatialAcceleration<double> A_ABp_A =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_B, p_BoBp_B, frame_A, frame_A);

  // Verify test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation()
  // results match by-hand result in CalcBpSpatialAccelerationInAExpressedInA().
  const SpatialAcceleration<double> A_ABp_A_expected =
      CalcBpSpatialAccelerationInAExpressedInA(wBzdot, p_BoBp_B);
  EXPECT_TRUE(CompareMatrices(A_ABp_A.get_coeffs(),
                              A_ABp_A_expected.get_coeffs(), kTolerance));
}

TEST_F(TwoDOFPlanarPendulumTest, CalcCenterOfMassAcceleration) {
  // Shortcuts to various frames.
  const Frame<double>& frame_W = plant_.world_frame();
  const Frame<double>& frame_A = bodyA_->body_frame();
  const Frame<double>& frame_B = bodyB_->body_frame();

  // Point Acm is link A's center of mass, colocated with Ao (A's centroid).
  // Calculate Acm's spatial acceleration in world W, expressed in world W.
  const Vector3<double> a_WAcm_W =
      bodyA_->CalcCenterOfMassTranslationalAccelerationInWorld(*context_);

  // Verify previous results by calculating Ao's spatial acceleration in W.
  const SpatialAcceleration<double> A_WAo_W_expected =
      frame_A.CalcSpatialAccelerationInWorld(*context_);
  EXPECT_TRUE(
      CompareMatrices(a_WAcm_W, A_WAo_W_expected.translational(), kTolerance));

  // Calculate generalized accelerations for use by test_utilities function.
  const Vector2<double> vdot = CalcVdotForNoAppliedForces();  // [ẇAz, ẇBz]

  // Verify results from CalcCenterOfMassTranslationalAccelerationInWorld()
  // match test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
  const Vector3<double> p_AoAcm_A = Vector3<double>::Zero();
  const SpatialAcceleration<double> A_WAcm_W_expected =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_A, p_AoAcm_A, frame_W, frame_W);
  EXPECT_TRUE(
      CompareMatrices(a_WAcm_W, A_WAcm_W_expected.translational(), kTolerance));

  // Point Bcm is link B's center of mass, colocated with Bo (B's centroid).
  // Calculate Bcm's spatial acceleration in world W, expressed in world W.
  const Vector3<double> a_WBcm_W =
      bodyB_->CalcCenterOfMassTranslationalAccelerationInWorld(*context_);

  // Verify previous results by calculating Bo's spatial acceleration in W.
  const SpatialAcceleration<double> A_WBo_W_expected =
      frame_B.CalcSpatialAccelerationInWorld(*context_);
  EXPECT_TRUE(
      CompareMatrices(a_WBcm_W, A_WBo_W_expected.translational(), kTolerance));

  // Verify results from CalcCenterOfMassTranslationalAccelerationInWorld()
  // match test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation().
  const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
  const SpatialAcceleration<double> A_WBcm_W_expected =
      test_utilities::CalcSpatialAccelerationViaAutomaticDifferentiation(
          plant_, *context_, vdot, frame_B, p_BoBcm_B, frame_W, frame_W);
  EXPECT_TRUE(
      CompareMatrices(a_WBcm_W, A_WBcm_W_expected.translational(), kTolerance));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
