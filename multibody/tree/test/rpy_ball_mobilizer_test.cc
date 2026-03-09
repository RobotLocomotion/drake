#include "drake/multibody/tree/rpy_ball_mobilizer.h"

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/ball_rpy_joint.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/test/mobilizer_tester.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Matrix3d;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;
using std::make_unique;
using std::unique_ptr;
using systems::Context;

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();

// Fixture to setup a simple MBT model containing a space xyz mobilizer.
class RpyBallMobilizerTest : public MobilizerTester {
 public:
  // Creates a simple model consisting of a single body with a space xyz
  // mobilizer connecting it to the world.
  void SetUp() override {
    mobilizer_ = &AddJointAndFinalize<BallRpyJoint, RpyBallMobilizer>(
        std::make_unique<BallRpyJoint<double>>(
            "joint0", tree().world_body().body_frame(), body_->body_frame()));
  }

 protected:
  const RpyBallMobilizer<double>* mobilizer_{nullptr};
};

TEST_F(RpyBallMobilizerTest, CanRotateOrTranslate) {
  EXPECT_TRUE(mobilizer_->can_rotate());
  EXPECT_FALSE(mobilizer_->can_translate());
}

// Verifies methods to mutate and access the context.
TEST_F(RpyBallMobilizerTest, StateAccess) {
  const Vector3d rpy_value(M_PI / 3, -M_PI / 3, M_PI / 5);
  mobilizer_->SetAngles(context_.get(), rpy_value);
  EXPECT_EQ(mobilizer_->get_angles(*context_), rpy_value);

  // Set mobilizer orientation using a rotation matrix.
  const RollPitchYawd rpy(M_PI / 5, -M_PI / 7, M_PI / 3);
  const RotationMatrixd R_WB(rpy);
  mobilizer_->SetFromRotationMatrix(context_.get(), R_WB);
  EXPECT_TRUE(CompareMatrices(mobilizer_->get_angles(*context_), rpy.vector(),
                              kTolerance, MatrixCompareType::relative));
}

TEST_F(RpyBallMobilizerTest, CalcAcrossMobilizerTransform) {
  const double kTol = 4 * std::numeric_limits<double>::epsilon();
  const Vector3d rpy_value(M_PI / 3, -M_PI / 3, M_PI / 5);
  mobilizer_->SetAngles(context_.get(), rpy_value);
  const double* q =
      &context_
           ->get_continuous_state_vector()[mobilizer_->position_start_in_q()];
  RigidTransformd X_FM(mobilizer_->CalcAcrossMobilizerTransform(*context_));

  const RigidTransformd X_FM_expected(RollPitchYawd(rpy_value),
                                      Vector3d::Zero());
  EXPECT_TRUE(X_FM.IsNearlyEqualTo(X_FM_expected, kTol));

  // Now check the fast inline methods.
  RigidTransformd fast_X_FM = mobilizer_->calc_X_FM(q);
  EXPECT_TRUE(fast_X_FM.IsNearlyEqualTo(X_FM, kTol));
  const Vector3d new_rpy_value(M_PI / 4, -M_PI / 4, M_PI / 7);
  mobilizer_->SetAngles(context_.get(), new_rpy_value);
  X_FM = mobilizer_->CalcAcrossMobilizerTransform(*context_);
  mobilizer_->update_X_FM(q, &fast_X_FM);
  EXPECT_TRUE(fast_X_FM.IsNearlyEqualTo(X_FM, kTol));

  TestApplyR_FM(X_FM, *mobilizer_);
  TestPrePostMultiplyByX_FM(X_FM, *mobilizer_);
}

TEST_F(RpyBallMobilizerTest, ZeroState) {
  // Set an arbitrary "non-zero" state.
  const Vector3d rpy_value(M_PI / 3, -M_PI / 3, M_PI / 5);
  mobilizer_->SetAngles(context_.get(), rpy_value);
  EXPECT_EQ(mobilizer_->get_angles(*context_), rpy_value);

  // Set the "zero state" for this mobilizer, which does happen to be that of
  // an identity rigid transform.
  mobilizer_->SetZeroState(*context_, &context_->get_mutable_state());
  const RigidTransformd X_WB(
      mobilizer_->CalcAcrossMobilizerTransform(*context_));
  EXPECT_TRUE(X_WB.IsExactlyIdentity());
}

// For an arbitrary state, verify that calculating the Nplus matrix N⁺(q) is the
// inverse of N(q), e.g., verify N⁺(q) * N(q) = [I] (the identity matrix). Also
// verify the time derivatives of N⁺(q) * N(q) and N(q) * N⁺(q) are both zero.
TEST_F(RpyBallMobilizerTest, KinematicMapping) {
  const Vector3d rpy(M_PI / 3, -M_PI / 3, M_PI / 5);
  mobilizer_->SetAngles(context_.get(), rpy);

  ASSERT_EQ(mobilizer_->num_positions(), 3);
  ASSERT_EQ(mobilizer_->num_velocities(), 3);

  // Compute the N(q) and Nplus(q) matrices.
  MatrixX<double> N(3, 3), Nplus(3, 3);
  mobilizer_->CalcNMatrix(*context_, &N);
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);

  // Verify that Nplus (N⁺) is the inverse of N and vice-versa.
  const MatrixX<double> N_x_Nplus = N * Nplus;
  const MatrixX<double> Nplus_x_N = Nplus * N;
  EXPECT_TRUE(CompareMatrices(N_x_Nplus, Matrix3d::Identity(), kTolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(Nplus_x_N, Matrix3d::Identity(), kTolerance,
                              MatrixCompareType::relative));

  // Also compare N(q) to numerical values produced by MotionGenesis.
  constexpr double epsilon64 = 64 * std::numeric_limits<double>::epsilon();
  MatrixX<double> Ncheck(3, 3);
  Ncheck.row(0) << 1.6180339887498949, 1.1755705045849463, 0;
  Ncheck.row(1) << -0.58778525229247314, 0.80901699437494745, 0;
  Ncheck.row(2) << -1.4012585384440732, -1.0180739209102541, 1;
  EXPECT_TRUE(
      CompareMatrices(N, Ncheck, epsilon64, MatrixCompareType::relative));

  // Set a generic angular velocity.
  const Vector3<double> v(0.5, -0.7, 2.3);
  mobilizer_->SetAngularVelocity(context_.get(), v);

  // Compute the NDot(q,q̇) and NplusDot(q,q̇) matrices.
  MatrixX<double> NDot(3, 3), NplusDot(3, 3);
  mobilizer_->CalcNDotMatrix(*context_, &NDot);
  mobilizer_->CalcNplusDotMatrix(*context_, &NplusDot);

  // The Nplus(q) matrix N⁺ multiplied by N(q) is the identity matrix [I].
  // Hence the time-derivative of (N⁺ * N = [I]) is Ṅ⁺ * N + N⁺ * Ṅ = [0],
  // where [0] is the zero matrix. Therefore Ṅ⁺ * N = -N⁺ * Ṅ.
  // Similarly, the time derivative of (N * N⁺ = [I]) is Ṅ * N⁺ + N * Ṅ⁺ = [0],
  // so Ṅ * N⁺ = -N * Ṅ⁺.  Verify these relationships.
  const MatrixX<double> NplusDot_N = NplusDot * N;
  const MatrixX<double> Nplus_NDot = Nplus * NDot;
  const MatrixX<double> NDot_Nplus = NDot * Nplus;
  const MatrixX<double> N_NplusDot = N * NplusDot;
  EXPECT_TRUE(CompareMatrices(NplusDot_N, -Nplus_NDot, kTolerance,
                              MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(NDot_Nplus, -N_NplusDot, kTolerance,
                              MatrixCompareType::relative));

  // Also compare Ṅ(q,q̇) to numerical values produced by MotionGenesis.
  MatrixX<double> NDotcheck(3, 3);
  NDotcheck.row(0) << -0.30720756492922385, 5.4924345293948527, 0;
  NDotcheck.row(1) << -1.8704654739876834, -1.358972714017757, 0;
  NDotcheck.row(2) << -0.42987052164155548, -5.2622033631883367, 0;
  EXPECT_TRUE(
      CompareMatrices(NDot, NDotcheck, epsilon64, MatrixCompareType::relative));

  // Ensure cos(pitch) ≈ 0, throws an exception.
  const double pitch = M_PI / 2 + epsilon64;
  const Vector3d rpy_singular_value(M_PI / 3, pitch, -M_PI / 5);
  mobilizer_->SetAngles(context_.get(), rpy_singular_value);
  DRAKE_EXPECT_THROWS_MESSAGE(mobilizer_->CalcNMatrix(*context_, &N),
                              "CalcNMatrix\\(\\): The RpyBallMobilizer .*"
                              "has reached a singularity.*");
  DRAKE_EXPECT_NO_THROW(mobilizer_->CalcNplusMatrix(*context_, &Nplus));
  DRAKE_EXPECT_THROWS_MESSAGE(mobilizer_->CalcNDotMatrix(*context_, &NDot),
                              "CalcNDotMatrix\\(\\): The RpyBallMobilizer .*"
                              "has reached a singularity.*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      mobilizer_->CalcNplusDotMatrix(*context_, &NplusDot),
      "CalcNplusDotMatrix\\(\\): The RpyBallMobilizer .*"
      "has reached a singularity.*");
}

TEST_F(RpyBallMobilizerTest, MapUsesN) {
  // Set an arbitrary "non-zero" state.
  const Vector3d rpy_value(M_PI / 3, -M_PI / 3, M_PI / 5);
  mobilizer_->SetAngles(context_.get(), rpy_value);

  EXPECT_FALSE(mobilizer_->is_velocity_equal_to_qdot());

  // Set arbitrary v and MapVelocityToQDot.
  const Vector3<double> v = (Vector3<double>() << 1, 2, 3).finished();
  Vector3<double> qdot;
  mobilizer_->MapVelocityToQDot(*context_, v, &qdot);

  // Compute N.
  MatrixX<double> N(3, 3);
  mobilizer_->CalcNMatrix(*context_, &N);

  // Ensure N(q) is used in q̇ = N(q)⋅v
  EXPECT_TRUE(
      CompareMatrices(qdot, N * v, kTolerance, MatrixCompareType::relative));
}

TEST_F(RpyBallMobilizerTest, MapUsesNplus) {
  // Set an arbitrary "non-zero" state.
  const Vector3d rpy_value(M_PI / 3, -M_PI / 3, M_PI / 5);
  mobilizer_->SetAngles(context_.get(), rpy_value);

  // Set arbitrary qdot and MapQDotToVelocity.
  const Vector3<double> qdot = (Vector3<double>() << 1, 2, 3).finished();
  Vector3<double> v;
  mobilizer_->MapQDotToVelocity(*context_, qdot, &v);

  // Compute Nplus.
  MatrixX<double> Nplus(3, 3);
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);

  // Ensure N⁺(q) is used in v = N⁺(q)⋅q̇
  EXPECT_TRUE(CompareMatrices(v, Nplus * qdot, kTolerance,
                              MatrixCompareType::relative));
}

TEST_F(RpyBallMobilizerTest, MapAccelerationToQDDotAndViceVersa) {
  // Set an arbitrary non-zero state.
  const Vector3<double> rpy(M_PI / 3, -M_PI / 4, M_PI / 5);
  const Vector3<double> wxyz(5.4, -9.8, 3.2);
  mobilizer_->SetAngles(context_.get(), rpy);
  mobilizer_->SetAngularVelocity(context_.get(), wxyz);

  // Set an arbitrary v̇ and use MapAccelerationToQDDot() to calculate q̈.
  const Vector3<double> vdot(0.3, -0.2, 0.9);  // v̇ = [ẇx, ẇy, ẇz].
  Vector3<double> qddot;
  mobilizer_->MapAccelerationToQDDot(*context_, vdot, &qddot);

  // Compute the 3x3 N(q) matrix and its time-derivative Ṅ(q,q̇).
  MatrixX<double> N(3, 3), Ndot(3, 3);
  mobilizer_->CalcNMatrix(*context_, &N);
  mobilizer_->CalcNDotMatrix(*context_, &Ndot);

  // Verify equivalence of q̈ = Ṅ(q,q̇)⋅v + N(q)⋅v̇ and MapAccelerationToQDDot().
  const Vector3<double> qddot_expected = Ndot * wxyz + N * vdot;
  EXPECT_TRUE(CompareMatrices(qddot, qddot_expected, kTolerance,
                              MatrixCompareType::relative));

  // Compute the 3x3 N⁺(q) matrix and its time-derivative Ṅ⁺(q,q̇).
  MatrixX<double> Nplus(3, 3), Nplusdot(3, 3);
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);
  mobilizer_->CalcNplusDotMatrix(*context_, &Nplusdot);

  // Starting with the previous q̈, use MapQDDotToAcceleration() to calculate v̇.
  Vector3<double> wdot;
  mobilizer_->MapQDDotToAcceleration(*context_, qddot, &wdot);

  // Verify equivalence of v̇ = Ṅ⁺(q,q̇)⋅q̇ + N⁺(q)⋅q̈ and MapQDDotToAcceleration().
  Vector3<double> qdot;
  mobilizer_->MapVelocityToQDot(*context_, wxyz, &qdot);
  const Vector3<double> vdot_expected = Nplusdot * qdot + Nplus * qddot;
  EXPECT_TRUE(CompareMatrices(vdot_expected, wdot, 16 * kTolerance,
                              MatrixCompareType::relative));

  // Verify MapQDDotToAcceleration() is the inverse of MapAccelerationToQDDot().
  EXPECT_TRUE(CompareMatrices(vdot, wdot, 16 * kTolerance,
                              MatrixCompareType::relative));
}

TEST_F(RpyBallMobilizerTest, SingularityError) {
  // Set state in singularity
  const Vector3d rpy_value(M_PI / 3, M_PI / 2, M_PI / 5);
  mobilizer_->SetAngles(context_.get(), rpy_value);

  // Set arbitrary qdot and MapVelocityToQDot.
  const Vector3<double> v = (Vector3<double>() << 1, 2, 3).finished();
  Vector3<double> qdot;
  DRAKE_EXPECT_THROWS_MESSAGE(
      mobilizer_->MapVelocityToQDot(*context_, v, &qdot), ".*singularity.*");

  // Compute N.
  MatrixX<double> N(3, 3);
  DRAKE_EXPECT_THROWS_MESSAGE(mobilizer_->CalcNMatrix(*context_, &N),
                              ".*singularity.*");
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
