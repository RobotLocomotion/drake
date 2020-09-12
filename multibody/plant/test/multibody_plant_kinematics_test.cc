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
// In the baseline configuration, the origin points Wo Ao Fo Mo Bo are
// sequential along the links (they form a line parallel to Wx = Ax = Bx).
class TwoDOFPlanarPendulumTest : public ::testing::Test {
 public:
  // Setup the MBP.
  void SetUp() override {
    // Set a spatial inertia for each link.  For now, these are unimportant
    // because this fixture is only used for kinematics tests.
    const double mass_link = 5.0;    // kg
    const double Izz = mass_link * link_length_ * link_length_ / 12.0;
    const RotationalInertia<double> I_BBcm(0, Izz, Izz);
    const Vector3<double> p_BoBcm_B = Vector3<double>::Zero();
    const SpatialInertia<double> M_Bcm =
        SpatialInertia<double>::MakeFromCentralInertia(mass_link, p_BoBcm_B,
                                                       I_BBcm);
    // Create an empty MultibodyPlant and then add the two links.
    plant_ = std::make_unique<MultibodyPlant<double>>(0.0);
    bodyA_ = &plant_->AddRigidBody("BodyA", M_Bcm);
    bodyB_ = &plant_->AddRigidBody("BodyB", M_Bcm);
    // Create a revolute joint that connects point Wo of the world frame to a
    // unnamed point of link A.  The revolute joint is a distance link_length/2
    // from link A's centroid (point Ao).
    const Vector3<double> p_AoWo_A(-0.5 * link_length_, 0.0, 0.0);
    joint1_ = &plant_->AddJoint<RevoluteJoint>("PinJoint1",
        plant_->world_body(), std::nullopt,
        *bodyA_, math::RigidTransformd(p_AoWo_A), Vector3<double>::UnitZ());
    // Create a revolute joint that connects point Fo (frame F's origin) to
    // point Mo (frame M's origin), where frame F is fixed/welded to the
    // distal end of link A (Fo is a distance of link_length/2 from Ao) and
    // frame M is fixed/welded to link B.  Mo is a distance of link_length/2
    // from link B's centroid (point Bo).
    const Vector3<double> p_AoFo_A(0.5 * link_length_, 0.0, 0.0);
    const Vector3<double> p_BoMo_B(-0.5 * link_length_, 0.0, 0.0);
    joint2_ = &plant_->AddJoint<RevoluteJoint>("PinJoint2",
        *bodyA_, math::RigidTransformd(p_AoFo_A),
        *bodyB_, math::RigidTransformd(p_BoMo_B), Vector3<double>::UnitZ());
    // Finalize the plant and create a context to store this plant's state.
    plant_->Finalize();
    context_ = plant_->CreateDefaultContext();
    // Scalar-convert the model and create a default context for it.
    plant_autodiff_ = systems::System<double>::ToAutoDiffXd(*plant_);
    context_autodiff_ = plant_autodiff_->CreateDefaultContext();
  }

 protected:
  // Since the maximum absolute value of acceleration in this test is
  // approximately ω² * (2 * link_length) ≈ 72 , we test that the errors in
  // acceleration calculations are less than 3 bits (2^3 = 8).
  const double kTolerance = 8 * std::numeric_limits<double>::epsilon();
  const double link_length_ = 4.0;  // meters
  const double wAz_ = 3.0;          // rad/sec
  const double wBz_ = 2.0;          // rad/sec
  std::unique_ptr<MultibodyPlant<double>> plant_;
  std::unique_ptr<systems::Context<double>> context_;
  const RigidBody<double>* bodyA_{nullptr};
  const RigidBody<double>* bodyB_{nullptr};
  const RevoluteJoint<double>* joint1_{nullptr};
  const RevoluteJoint<double>* joint2_{nullptr};

  // AutoDiffXd this system to automate derivative calculations.
  std::unique_ptr<MultibodyPlant<AutoDiffXd>> plant_autodiff_;
  std::unique_ptr<systems::Context<AutoDiffXd>> context_autodiff_;
};

// This test and the next test show how the utility function
// test_utilities::CalcSpatialAccelerationViaSpatialVelocityDerivative()
// can be reliably used to test MultibodyPlant::CalcBiasSpatialAcceleration()
// and MultibodyPlant::CalcSpatialAcceleration().
TEST_F(TwoDOFPlanarPendulumTest, CalcBiasSpatialAccelerationViaVelDerivative) {
  Eigen::VectorXd state = Eigen::Vector4d(0.0, 0.0, wAz_, wBz_);
  joint1_->set_angle(context_.get(), state[0]);
  joint2_->set_angle(context_.get(), state[1]);
  joint1_->set_angular_rate(context_.get(), state[2]);
  joint2_->set_angular_rate(context_.get(), state[3]);
  const Eigen::VectorXd vDt = Eigen::Vector2d(0.0, 0.0);

  // Point Ap is the point of A located at the revolute joint connecting link A
  // and link B.  Calculate Ap's bias spatial acceleration in world W.
  const Frame<double>& frame_A = bodyA_->body_frame();
  const Frame<double>& frame_W = plant_->world_frame();
  const Vector3<double> p_AoAp_A(0.5 * link_length_, 0.0, 0.0);
  const SpatialAcceleration<double> aBias_WAp_W =
      test_utilities::CalcSpatialAccelerationViaSpatialVelocityDerivative(
        *plant_, *context_, vDt, frame_A, p_AoAp_A, frame_W, frame_W);

  // Simple by-hand analysis gives aBias_WAp_W = -L wAz_² Wx.
  const double wA_squared = wAz_ * wAz_;
  const Vector3<double> aBias_WAp_W_expected = -link_length_ *
      wA_squared * Vector3d::UnitX();
  EXPECT_TRUE(CompareMatrices(aBias_WAp_W.translational(), aBias_WAp_W_expected,
                              kTolerance));
  EXPECT_TRUE(CompareMatrices(aBias_WAp_W.rotational(), Vector3d::Zero(),
                              kTolerance));
#if 0
  // Also show that this utility function is useful for testing the method
  // MultibodyPlant::CalcBiasSpatialAcceleration().
  const SpatialAcceleration<double> aBias_WAp_W_alternate =
    plant_->CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                        frame_A, p_AoAp_A, frame_W, frame_W);
  EXPECT_TRUE(CompareMatrices(aBias_WAp_W_alternate.get_coeffs(),
                              aBias_WAp_W.get_coeffs(), kTolerance));
#endif

  // Point Bp is the point of B located at the most distal end of link B.
  // Calculate Bp's bias spatial acceleration in world W.
  const Frame<double>& frame_B = bodyB_->body_frame();
  const Vector3<double> p_BoBp_B(0.5 * link_length_, 0.0, 0.0);
  const SpatialAcceleration<double> aBias_WBp_W =
      test_utilities::CalcSpatialAccelerationViaSpatialVelocityDerivative(
        *plant_, *context_, vDt, frame_B, p_BoBp_B, frame_W, frame_W);

  // By-hand analysis gives aBias_WBp_W = -L wAz_² Wx - L (wAz_ + wBz_)² Wx
  const double wAB_squared = (wAz_ + wBz_) * (wAz_ + wBz_);
  const Vector3<double> aBias_WBp_W_expected = (-link_length_ * wA_squared +
      -link_length_ * wAB_squared) * Vector3d::UnitX();
  EXPECT_TRUE(CompareMatrices(aBias_WBp_W.translational(), aBias_WBp_W_expected,
                              kTolerance));
  EXPECT_TRUE(CompareMatrices(aBias_WBp_W.rotational(), Vector3d::Zero(),
                              kTolerance));

#if 0
  // Also show that this utility function is useful for testing the method
  // MultibodyPlant::CalcBiasSpatialAcceleration().
  const SpatialAcceleration<double> aBias_WBp_W_alternate =
      plant_->CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                          frame_A, p_AoAp_A, frame_W, frame_W);
  EXPECT_TRUE(CompareMatrices(aBias_WBp_W_alternate.get_coeffs(),
                              aBias_WBp_W.get_coeffs(), kTolerance));
#endif

  // For point Ap of A, calculate Ap's bias spatial acceleration in frame A.
  // Simple by-hand analysis gives aBias_AAp_A = Vector3d::Zero().
  const SpatialAcceleration<double> aBias_AAp_A =
      test_utilities::CalcSpatialAccelerationViaSpatialVelocityDerivative(
          *plant_, *context_, vDt, frame_A, p_AoAp_A, frame_A, frame_A);
  EXPECT_TRUE(CompareMatrices(aBias_AAp_A.translational(), Vector3d::Zero(),
      kTolerance));
  EXPECT_TRUE(CompareMatrices(aBias_AAp_A.rotational(), Vector3d::Zero(),
      kTolerance));

#if 0
  // Also show that this utility function is useful for testing the method
  // MultibodyPlant::CalcBiasSpatialAcceleration().
  const SpatialAcceleration<double> aBias_AAp_A_alternate =
    plant_->CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                        frame_A, p_AoAp_A, frame_W, frame_W);
  EXPECT_TRUE(CompareMatrices(aBias_AAp_A_alternate.get_coeffs(),
                              aBias_AAp_A.get_coeffs(), kTolerance));
#endif

  // For point Bp of B, calculate Bp's bias spatial acceleration in frame A.
  const SpatialAcceleration<double> aBias_ABp_A =
      test_utilities::CalcSpatialAccelerationViaSpatialVelocityDerivative(
          *plant_, *context_, vDt, frame_B, p_BoBp_B, frame_A, frame_A);

  // By-hand analysis gives aBias_ABp_A = -L wBz_² Ax.
  const Vector3<double> aBias_ABp_A_expected =
      -link_length_ * wBz_ * wBz_ * Vector3d::UnitX();
  EXPECT_TRUE(CompareMatrices(aBias_ABp_A.translational(), aBias_ABp_A_expected,
                              kTolerance));
  EXPECT_TRUE(CompareMatrices(aBias_ABp_A.rotational(), Vector3d::Zero(),
                              kTolerance));
#if 0
  // Also show that this utility function is useful for testing the method
  // MultibodyPlant::CalcBiasSpatialAcceleration().
  const SpatialAcceleration<double> aBias_ABp_A_alternate =
    plant_->CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                        frame_A, p_AoAp_A, frame_W, frame_W);
  EXPECT_TRUE(CompareMatrices(aBias_ABp_A_alternate.get_coeffs(),
                              aBias_ABp_A.get_coeffs(), kTolerance));
#endif

  // Point Bq is fixed to link B and located from Bo as below.
  const double x = 0.1, y = -0.2, z = 0.3;
  const Vector3<double> p_BoBq_B(x, y, z);

  // Compute Bq's bias translational acceleration with respect to generalized
  // velocities v measured in frame A, expressed in frame B.
  const SpatialAcceleration<double> aBias_ABq_B =
      test_utilities::CalcSpatialAccelerationViaSpatialVelocityDerivative(
          *plant_, *context_, vDt, frame_B, p_BoBq_B, frame_A, frame_B);

  // MotionGenesis gives  aBias_ABq_B = -0.5 (L+2*x) wBz_² Bx -   y wBz_² By.
  const double wB_squared = wBz_ * wBz_;
  const Vector3<double> aBias_ABq_B_expected(
      -0.5 * (link_length_ + 2 * x) * wB_squared, -y * wB_squared, 0);
  EXPECT_TRUE(CompareMatrices(aBias_ABq_B.translational(),
                              aBias_ABq_B_expected,  kTolerance));

#if 0
  // Also show that this utility function is useful for testing the method
  // MultibodyPlant::CalcBiasSpatialAcceleration().
  const SpatialAcceleration<double> aBias_ABq_B_alternate =
    plant_->CalcBiasSpatialAcceleration(*context_, JacobianWrtVariable::kV,
                                        frame_A, p_AoAp_A, frame_W, frame_W);
  EXPECT_TRUE(CompareMatrices(aBias_ABq_B_alternate.get_coeffs(),
                              aBias_ABq_B.get_coeffs(), kTolerance));
#endif
}

// This test and the previous test show how the utility function
// test_utilities::CalcSpatialAccelerationViaSpatialVelocityDerivative()
// can be reliably used to test MultibodyPlant::CalcBiasSpatialAcceleration()
// and MultibodyPlant::CalcSpatialAcceleration().
TEST_F(TwoDOFPlanarPendulumTest, CalcSpatialAccelerationViaVectorDerivative) {
  const double qA = M_PI / 6.0, qB = 0;  // Link angles.
  const double wAzDt = 10, wBzDt = 20;   // Link angular accelerations.
  joint1_->set_angle(context_.get(), qA);
  joint2_->set_angle(context_.get(), qB);
  joint1_->set_angular_rate(context_.get(), wAz_);
  joint2_->set_angular_rate(context_.get(), wBz_);
  const Eigen::VectorXd vDt = Eigen::Vector2d(wAzDt, wBzDt);

  // Shortcuts to various frames.
  const Frame<double>& frame_W = plant_->world_frame();
  const Frame<double>& frame_A = bodyA_->body_frame();
  const Frame<double>& frame_B = bodyB_->body_frame();

  // Ensure frame A's spatial acceleration in frame A is zero.
  const Vector3<AutoDiffXd> p_AoAo_A = Vector3<AutoDiffXd>::Zero();
  const SpatialAcceleration<double> A_AAo_A =
      test_utilities::CalcSpatialAccelerationViaSpatialVelocityDerivative(
          *plant_, *context_, vDt, frame_A, frame_A, frame_A);
  EXPECT_TRUE(CompareMatrices(A_AAo_A.rotational(), Vector3<double>::Zero(),
                              kTolerance));
  EXPECT_TRUE(CompareMatrices(A_AAo_A.translational(), Vector3<double>::Zero(),
                              kTolerance));

  // Calculate frame A's spatial acceleration in frame W, expressed in A.
  const SpatialAcceleration<double> A_WAo_A =
      test_utilities::CalcSpatialAccelerationViaSpatialVelocityDerivative(
          *plant_, *context_, vDt, frame_A, frame_W, frame_A);

  // Compare results with the following by-hand analytical results.
  // DtW(w_WA_A) = DtA(w_WA_A) + w_WA_A x w_WA_A
  //             = Dt(wAz) Az
  //             = alphaA Az
  // DtW(v_WA_A) = DtA(v_WA_A) + w_WA_A x v_WA_A
  //             = 0.5 L alphaA Ay + wAz Az x 0.5 L wAz Ay.
  //             = 0.5 L alphaA Ay - 0.5 L wAz² Ax
  const double alphaA = wAzDt;
  const double wA_squared = wAz_ * wAz_;
  const Vector3<double> alpha_WA_A_expected(0, 0, alphaA);
  const Vector3<double> a_WA_A_expected(
      -0.5 * link_length_ * wA_squared, 0.5 * link_length_ * alphaA, 0);
  EXPECT_TRUE(
      CompareMatrices(A_WAo_A.rotational(), alpha_WA_A_expected, kTolerance));
  EXPECT_TRUE(
      CompareMatrices(A_WAo_A.translational(), a_WA_A_expected, kTolerance));

  // Ensure frame B's spatial acceleration in frame B is zero.
  const SpatialAcceleration<double> A_BBo_W =
      test_utilities::CalcSpatialAccelerationViaSpatialVelocityDerivative(
          *plant_, *context_, vDt, frame_B, frame_B, frame_W);
  EXPECT_TRUE(CompareMatrices(A_BBo_W.rotational(), Vector3<double>::Zero(),
                              kTolerance));
  EXPECT_TRUE(CompareMatrices(A_BBo_W.translational(), Vector3<double>::Zero(),
                              kTolerance));

  // Calculate frame B's spatial acceleration in frame W, expressed in A.
  const SpatialAcceleration<double> A_WBo_A =
      test_utilities::CalcSpatialAccelerationViaSpatialVelocityDerivative(
          *plant_, *context_, vDt, frame_B, frame_W, frame_A);

  // Compare results with the following by-hand analytical results.
  // DtW(w_WB_A) = Dt(wAz + wBz) Az
  //             = (wAzDt + wBzDt) Az
  //             = alphaAB Az
  // A_WBo_W =  L [alphaA Ay - wAz² Ax + 0.5 alphaAB By - 0.5 (wAz + wBz)² Bx]
  //         = -L [wAz² + 0.5 (wAz + wBz)²] Ax + L (alphaA + 0.5 alphaAB) Ay
  // Reminder: Ax = Bx, Ay = By, Az = Bz when qB = 0°.
  const double alphaAB = wAzDt + wBzDt;
  const double wAB_squared = (wAz_ + wBz_) * (wAz_ + wBz_);
  const Vector3<double> alpha_WB_A_expected(0, 0, alphaAB);
  const Vector3<double> a_WBo_A_expected(
      -link_length_ * (wA_squared + 0.5 * wAB_squared),
       link_length_ * (alphaA + 0.5 * alphaAB), 0);
  EXPECT_TRUE(
      CompareMatrices(A_WBo_A.rotational(), alpha_WB_A_expected, kTolerance));
  EXPECT_TRUE(
      CompareMatrices(A_WBo_A.translational(), a_WBo_A_expected, kTolerance));

  // Designate frame_Ap as the frame fixed to A that is joint2's parent frame.
  // Calculate frame_Ap's spatial acceleration in frame W, expressed in A.
  const Frame<double>& frame_Ap = joint2_->frame_on_parent();
  const SpatialAcceleration<double> A_WAp_A =
      test_utilities::CalcSpatialAccelerationViaSpatialVelocityDerivative(
          *plant_, *context_, vDt, frame_Ap, frame_W, frame_A);

  // Compare results with the following by-hand analytical results.
  // DtW(w_WA_A)  = alphaA Az
  // DtW(v_WAp_A) = DtA(v_WAp_A) + w_WA_A x v_WAp_A
  //              = L alphaA Ay + wAz Az x L wAz Ay.
  //              = L alphaA Ay - L wAz² Ax
  const Vector3<double> a_WAp_A_expected(
      -link_length_ * wA_squared, link_length_ * alphaA, 0);
  EXPECT_TRUE(
      CompareMatrices(A_WAp_A.rotational(), alpha_WA_A_expected, kTolerance));
  EXPECT_TRUE(
      CompareMatrices(A_WAp_A.translational(), a_WAp_A_expected, kTolerance));

  // Calculate frame_Bp's spatial acceleration in frame W, expressed in A.
  // Note: frame_Bp is the frame fixed to bodyB_'s distal end.
  const Vector3<double> p_BoBp_B(0.5 * link_length_, 0, 0);
  const SpatialAcceleration<double> A_WBp_A =
      test_utilities::CalcSpatialAccelerationViaSpatialVelocityDerivative(
          *plant_, *context_, vDt, frame_B, p_BoBp_B, frame_W, frame_A);

  // Compare results with the following by-hand analytical results.
  // A_WBp_W = L alphaA Ay - L wAz² Ax + L alphaAB By - L (wAz + wBz)² Bx
  //         = -L [wAz² + (wAz + wBz)²] Ax  + L (alphaA + alphaAB) Ay
  // Reminder: Ax = Bx, Ay = By, Az = Bz when qB = 0°.
  const Vector3<double> a_WBp_A_expected(
      -link_length_ * (wA_squared + wAB_squared),
      link_length_ * (alphaA + alphaAB), 0);
  EXPECT_TRUE(
      CompareMatrices(A_WBp_A.translational(), a_WBp_A_expected, kTolerance));

  // Calculate frame_Bp's spatial acceleration in frame A, expressed in A.
  const SpatialAcceleration<double> A_ABp_A =
      test_utilities::CalcSpatialAccelerationViaSpatialVelocityDerivative(
          *plant_, *context_, vDt, frame_B, p_BoBp_B, frame_A, frame_A);

  // Compare results with the following by-hand analytical results.
  // DtA(w_AB_A)  = Dt(wBz) Az
  //              = alphaB Az
  // DtA(v_ABp_A) = DtB(v_ABp_A) + w_AB_A x v_ABp_A
  //              = L alphaB By + wBz Bz x L wBz By.
  //              = L alphaB By - L wBz² Bx
  // Reminder: Ax = Bx, Ay = By, Az = Bz when qB = 0°.
  const double alphaB = wBzDt;
  const double wB_squared = wBz_ * wBz_;
  const Vector3<double> alpha_AB_A_expected(0, 0, alphaB);
  const Vector3<double> a_ABp_A_expected(
      -link_length_ * wB_squared, link_length_ * alphaB, 0);
  EXPECT_TRUE(CompareMatrices(
      A_ABp_A.rotational(), alpha_AB_A_expected, kTolerance));
  EXPECT_TRUE(CompareMatrices(
      A_ABp_A.translational(), a_ABp_A_expected, 16 * kTolerance));
}

}  // namespace
}  // namespace multibody
}  // namespace drake
