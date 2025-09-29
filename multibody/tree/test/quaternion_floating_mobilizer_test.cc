#include "drake/multibody/tree/quaternion_floating_mobilizer.h"

#include <limits>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/quaternion.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/quaternion_floating_joint.h"
#include "drake/multibody/tree/test/mobilizer_tester.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Quaterniond;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;
using std::make_unique;
using std::unique_ptr;

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();

// Fixture to setup a simple MBT model containing a quaternion mobilizer.
class QuaternionFloatingMobilizerTest : public MobilizerTester {
 public:
  void SetUp() override {
    mobilizer_ = &AddJointAndFinalize<QuaternionFloatingJoint,
                                      QuaternionFloatingMobilizer>(
        std::make_unique<QuaternionFloatingJoint<double>>(
            "joint0", tree().world_body().body_frame(), body_->body_frame()));
    mutable_mobilizer_ =
        const_cast<QuaternionFloatingMobilizer<double>*>(mobilizer_);
  }

 protected:
  const QuaternionFloatingMobilizer<double>* mobilizer_{nullptr};
  QuaternionFloatingMobilizer<double>* mutable_mobilizer_{nullptr};
};

TEST_F(QuaternionFloatingMobilizerTest, CanRotateOrTranslate) {
  EXPECT_TRUE(mobilizer_->can_rotate());
  EXPECT_TRUE(mobilizer_->can_translate());
}

// Verifies methods to mutate and access the context.
TEST_F(QuaternionFloatingMobilizerTest, StateAccess) {
  const Quaterniond quaternion_value(
      RollPitchYawd(M_PI / 3, -M_PI / 3, M_PI / 5).ToQuaternion());
  mobilizer_->SetQuaternion(context_.get(), quaternion_value);
  EXPECT_EQ(mobilizer_->get_quaternion(*context_).coeffs(),
            quaternion_value.coeffs());

  const Vector3d translation_value(1.0, 2.0, 3.0);
  mobilizer_->SetTranslation(context_.get(), translation_value);
  EXPECT_EQ(mobilizer_->get_translation(*context_), translation_value);

  // Set mobilizer orientation using a rotation matrix.
  const RotationMatrixd R_WB(RollPitchYawd(M_PI / 5, -M_PI / 7, M_PI / 3));
  const Quaterniond Q_WB = R_WB.ToQuaternion();
  mobilizer_->SetOrientation(context_.get(), R_WB);
  EXPECT_TRUE(CompareMatrices(mobilizer_->get_quaternion(*context_).coeffs(),
                              Q_WB.coeffs(), kTolerance,
                              MatrixCompareType::relative));
}

TEST_F(QuaternionFloatingMobilizerTest, ZeroState) {
  // Set an arbitrary "non-zero" state.
  const Quaterniond quaternion_value(
      RollPitchYawd(M_PI / 3, -M_PI / 3, M_PI / 5).ToQuaternion());
  mobilizer_->SetQuaternion(context_.get(), quaternion_value);
  EXPECT_EQ(mobilizer_->get_quaternion(*context_).coeffs(),
            quaternion_value.coeffs());

  // Set the "zero state" for this mobilizer, which does happen to be that of
  // an identity rigid transform.
  mobilizer_->SetZeroState(*context_, &context_->get_mutable_state());
  const RigidTransformd X_WB(
      mobilizer_->CalcAcrossMobilizerTransform(*context_));
  EXPECT_TRUE(X_WB.IsExactlyIdentity());
}

TEST_F(QuaternionFloatingMobilizerTest, CalcAcrossMobilizerTransform) {
  const double kTol = 4 * std::numeric_limits<double>::epsilon();
  // Set an arbitrary "non-zero" state.
  const Quaterniond quaternion(
      RollPitchYawd(M_PI / 3, -M_PI / 3, M_PI / 5).ToQuaternion());
  const Vector3d translation(1.0, 2.0, 3.0);
  mobilizer_->SetQuaternion(context_.get(), quaternion);
  mobilizer_->SetTranslation(context_.get(), translation);
  const double* q =
      &context_
           ->get_continuous_state_vector()[mobilizer_->position_start_in_q()];
  RigidTransformd X_FM(mobilizer_->CalcAcrossMobilizerTransform(*context_));

  const RigidTransformd X_FM_expected(quaternion, translation);
  EXPECT_TRUE(X_FM.IsNearlyEqualTo(X_FM_expected, kTol));

  // Now check the fast inline methods.
  RigidTransformd fast_X_FM = mobilizer_->calc_X_FM(q);
  EXPECT_TRUE(fast_X_FM.IsNearlyEqualTo(X_FM, kTol));
  const Quaterniond new_quaternion(
      RollPitchYawd(M_PI / 4, -M_PI / 4, M_PI / 7).ToQuaternion());
  const Vector3d new_translation(1.5, 2.5, 3.5);
  mobilizer_->SetQuaternion(context_.get(), new_quaternion);
  mobilizer_->SetTranslation(context_.get(), new_translation);
  X_FM = mobilizer_->CalcAcrossMobilizerTransform(*context_);
  mobilizer_->update_X_FM(q, &fast_X_FM);
  EXPECT_TRUE(fast_X_FM.IsNearlyEqualTo(X_FM, kTol));

  TestApplyR_FM(X_FM, *mobilizer_);
  TestPrePostMultiplyByX_FM(X_FM, *mobilizer_);
}

// Our documentation guarantees that this joint will represent a
// (quaternion, translation) pair exactly. Make sure it does.
TEST_F(QuaternionFloatingMobilizerTest, SetGetPosePair) {
  const Quaterniond set_quaternion(RollPitchYawd(0.1, 0.2, 0.3).ToQuaternion());
  const Vector3d set_translation(1.0, 2.0, 3.0);
  const RigidTransformd set_pose(set_quaternion, set_translation);

  // Make sure we don't accidentally match.
  const std::pair<Quaterniond, Vector3d> before =
      mobilizer_->GetPosePair(*context_);
  EXPECT_FALSE(math::RigidTransform(before.first, before.second)
                   .IsNearlyEqualTo(set_pose, 1e-8));

  mobilizer_->SetPosePair(*context_, set_quaternion, set_translation,
                          &context_->get_mutable_state());

  const std::pair<Quaterniond, Vector3d> after =
      mobilizer_->GetPosePair(*context_);

  // Check for bit-identical match.
  EXPECT_EQ(after.first.coeffs(), set_quaternion.coeffs());
  EXPECT_EQ(after.second, set_translation);
}

TEST_F(QuaternionFloatingMobilizerTest, SetGetSpatialVelocity) {
  const SpatialVelocity<double> set_V(Vector3d(1.0, 2.0, 3.0),
                                      Vector3d(4.0, 5.0, 6.0));

  // Make sure we don't accidentally match.
  const SpatialVelocity<double> before =
      mobilizer_->GetSpatialVelocity(*context_);
  EXPECT_FALSE(before.IsApprox(set_V, 1e-8));

  mobilizer_->SetSpatialVelocity(*context_, set_V,
                                 &context_->get_mutable_state());

  const SpatialVelocity<double> after =
      mobilizer_->GetSpatialVelocity(*context_);

  // We don't promise, but this should be a bit-identical match.
  EXPECT_EQ(after.get_coeffs(), set_V.get_coeffs());
}

TEST_F(QuaternionFloatingMobilizerTest, RandomState) {
  RandomGenerator generator;
  std::uniform_real_distribution<symbolic::Expression> uniform;

  // Default behavior is to set to zero.
  mutable_mobilizer_->set_random_state(
      *context_, &context_->get_mutable_state(), &generator);
  EXPECT_TRUE(
      RigidTransformd(mobilizer_->CalcAcrossMobilizerTransform(*context_))
          .IsExactlyIdentity());
  EXPECT_TRUE(mobilizer_->get_translation(*context_).isZero());
  EXPECT_TRUE(mobilizer_->get_angular_velocity(*context_).isZero());
  EXPECT_TRUE(mobilizer_->get_translational_velocity(*context_).isZero());

  Eigen::Matrix<symbolic::Expression, 3, 1> translation_distribution;
  for (int i = 0; i < 3; i++) {
    translation_distribution[i] = uniform(generator) + i + 1.0;
  }

  // Set position to be random, but not velocity (yet).
  mutable_mobilizer_->set_random_quaternion_distribution(
      math::UniformlyRandomQuaternion<symbolic::Expression>(&generator));
  mutable_mobilizer_->set_random_translation_distribution(
      translation_distribution);
  mutable_mobilizer_->set_random_state(
      *context_, &context_->get_mutable_state(), &generator);
  EXPECT_FALSE(
      RigidTransformd(mobilizer_->CalcAcrossMobilizerTransform(*context_))
          .IsExactlyIdentity());
  EXPECT_FALSE(mobilizer_->get_translation(*context_).isZero());
  EXPECT_TRUE(mobilizer_->get_angular_velocity(*context_).isZero());
  EXPECT_TRUE(mobilizer_->get_translational_velocity(*context_).isZero());

  // Set the velocity distribution.  Now both should be random.
  Eigen::Matrix<symbolic::Expression, 6, 1> velocity_distribution;
  for (int i = 0; i < 6; i++) {
    velocity_distribution[i] = uniform(generator) - i - 1.0;
  }
  mutable_mobilizer_->set_random_velocity_distribution(velocity_distribution);
  mutable_mobilizer_->set_random_state(
      *context_, &context_->get_mutable_state(), &generator);
  EXPECT_FALSE(
      RigidTransformd(mobilizer_->CalcAcrossMobilizerTransform(*context_))
          .IsExactlyIdentity());
  EXPECT_FALSE(mobilizer_->get_translation(*context_).isZero());
  EXPECT_FALSE(mobilizer_->get_angular_velocity(*context_).isZero());
  EXPECT_FALSE(mobilizer_->get_translational_velocity(*context_).isZero());
}

// Verify various properties of the N(q), N⁺(q), Ṅ⁺(q,q̇), Ṅ⁺(q,q̇) matrices.
TEST_F(QuaternionFloatingMobilizerTest, KinematicMapping) {
  ASSERT_EQ(mobilizer_->num_positions(), 7);
  ASSERT_EQ(mobilizer_->num_velocities(), 6);

  // Set an arbitrary orientation and position for this mobilizer.
  const Quaternion<double> Q_FM(
      RollPitchYawd(M_PI / 3, -M_PI / 3, M_PI / 5).ToQuaternion());
  mobilizer_->SetQuaternion(context_.get(), Q_FM);
  const Vector3<double> p_FoMo_F(1.0, 2.0, 3.0);
  mobilizer_->SetTranslation(context_.get(), p_FoMo_F);

  // Calculate the N matrix that appears in q̇ = N(q)⋅v.
  MatrixX<double> N(7, 6);
  mobilizer_->CalcNMatrix(*context_, &N);

  // Calculate the Nplus matrix that appears in v = N⁺(q)⋅q̇.
  MatrixX<double> Nplus(6, 7);
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);

  // Ensure the N⁺(q) matrix is the left pseudoinverse of the N(q) matrix.
  // In other words, ensure Nplus * N = [I₆₆] (6x6 identity matrix).
  MatrixX<double> Nplus_x_N = Nplus * N;
  EXPECT_TRUE(CompareMatrices(Nplus_x_N, MatrixX<double>::Identity(6, 6),
                              kTolerance, MatrixCompareType::relative));

  // Ensure the rotation (upper-left block) part of the N(q) matrix is
  // 0.25 times the transpose of the rotation part of the N⁺(q) matrix.
  MatrixX<double> N_rotational = N.block<4, 3>(0, 0);
  MatrixX<double> Nplus_rotational = Nplus.block<3, 4>(0, 0);
  EXPECT_TRUE(CompareMatrices(N_rotational.transpose(), 0.25 * Nplus_rotational,
                              kTolerance, MatrixCompareType::relative));

  // An arbitrary orientation and position (q) for this mobilizer was set above.
  // Set an arbitrary angular and translational velocity (v) for this mobilizer
  // to enable testing of Ṅ(q,q̇) and Ṅ⁺(q,q̇).
  const Vector3<double> w_FM_F(1.1, 2.5, 3.2);
  mobilizer_->SetAngularVelocity(context_.get(), w_FM_F);
  const Vector3<double> v_FMo_F(1.0, 2.0, 3.0);
  mobilizer_->SetTranslationalVelocity(context_.get(), v_FMo_F);

  // Calculate the NDot(q,q̇) matrix that appears in q̈ = Ṅ(q,q̇)⋅v + N⁺(q)⋅v̇.
  MatrixX<double> NDot(7, 6);
  mobilizer_->CalcNDotMatrix(*context_, &NDot);

  // Use the given data to calculate the expected values in the Ṅ(q,q̇) matrix.
  // Get the quaternion's scalar portion (eo) and vector portion (evec).
  const double e0 = Q_FM.w();  // Scalar portion of the quaternion.
  const Vector3<double> evec(Q_FM.x(), Q_FM.y(), Q_FM.z());  // Vector portion.

  // Use Eq. 5 in §9.3 of Mitiguy, Advanced Dynamics & Motion Simulation, 2019,
  // to calculate ė₀, the time-derivative of evec in frame F, and NDot(q,q̇).
  const double e0dot = -0.5 * evec.dot(w_FM_F);
  const Vector3<double> evec_dtA = 0.5 * (e0 * w_FM_F - evec.cross(w_FM_F));
  const double e1dot = evec_dtA.x();
  const double e2dot = evec_dtA.y();
  const double e3dot = evec_dtA.z();
  // clang-format off
  MatrixX<double> NDot_rotation_expected = 0.5 * (Eigen::Matrix<double, 4, 3>()
      << -e1dot, -e2dot, -e3dot,
          e0dot,  e3dot, -e2dot,
         -e3dot,  e0dot,  e1dot,
          e2dot, -e1dot,  e0dot).finished();
  // clang-format on

  // Ensure the calculation of the rotational part of the Ṅ(q,q̇) matrix
  // matches the by-hand calculations above.
  MatrixX<double> NDot_rotation = NDot.block<4, 3>(0, 0);
  EXPECT_TRUE(CompareMatrices(NDot_rotation, NDot_rotation_expected, kTolerance,
                              MatrixCompareType::relative));

  // Create the full (rotational and translational) expected Ṅ(q,q̇) matrix.
  // Note: All its elements are zero except some associated with rotation.
  MatrixX<double> NDot_expected(7, 6);
  NDot_expected.setZero();
  NDot_expected.template block<4, 3>(0, 0) = NDot_rotation;  // Upper-left.

  // Ensure the Drake calculation of Ndot matches the by-hand calculations.
  EXPECT_TRUE(CompareMatrices(NDot, NDot_expected, kTolerance,
                              MatrixCompareType::relative));

  // Calculate the NplusDot(q,q̇) matrix that appears in v̇ = Ṅ⁺(q,q̇)⋅v + N⁺(q)⋅q̈.
  MatrixX<double> NplusDot(6, 7);
  mobilizer_->CalcNplusDotMatrix(*context_, &NplusDot);

  // Since N⁺(q) * N(q) = [I₆₆] (the 6x6 identity matrix), then
  // Ṅ⁺(q,q̇) * N(q) + N⁺(q) * Ṅ(q,q̇) = [0₆₆] (the 6x6 zero matrix).
  MatrixX<double> zero_matrix_expected = NplusDot * N + Nplus * NDot;
  EXPECT_TRUE(CompareMatrices(zero_matrix_expected, MatrixX<double>::Zero(6, 6),
                              kTolerance, MatrixCompareType::relative));
}

TEST_F(QuaternionFloatingMobilizerTest, CheckExceptionMessage) {
  const Quaterniond quaternion(0, 0, 0, 0);
  mobilizer_->SetQuaternion(context_.get(), quaternion);

  const Vector3d translation(0, 0, 0);
  mobilizer_->SetTranslation(context_.get(), translation);

  DRAKE_EXPECT_THROWS_MESSAGE(
      mobilizer_->CalcAcrossMobilizerTransform(*context_),
      "QuaternionToRotationMatrix\\(\\):"
      " All the elements in a quaternion are zero\\.");
}

TEST_F(QuaternionFloatingMobilizerTest, MapUsesN) {
  // Set an arbitrary "non-zero" state.
  const Quaterniond Q_WB(
      RollPitchYawd(M_PI / 3, -M_PI / 3, M_PI / 5).ToQuaternion());
  mobilizer_->SetQuaternion(context_.get(), Q_WB);

  const Vector3d p_WB(1.0, 2.0, 3.0);
  mobilizer_->SetTranslation(context_.get(), p_WB);

  EXPECT_FALSE(mobilizer_->is_velocity_equal_to_qdot());

  // Set arbitrary v and MapVelocityToQDot
  const Vector6<double> v =
      (Vector6<double>() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished();
  VectorX<double> qdot(7);
  mobilizer_->MapVelocityToQDot(*context_, v, &qdot);

  // Compute N.
  MatrixX<double> N(7, 6);
  mobilizer_->CalcNMatrix(*context_, &N);

  // Ensure N(q) is used in `q̇ = N(q)⋅v`
  EXPECT_TRUE(
      CompareMatrices(qdot, N * v, kTolerance, MatrixCompareType::relative));
}

TEST_F(QuaternionFloatingMobilizerTest, MapUsesNplus) {
  // Set an arbitrary "non-zero" state.
  const Quaterniond q_WB(
      RollPitchYawd(M_PI / 3, -M_PI / 3, M_PI / 5).ToQuaternion());
  mobilizer_->SetQuaternion(context_.get(), q_WB);

  const Vector3d p_WB(1.0, 2.0, 3.0);
  mobilizer_->SetTranslation(context_.get(), p_WB);

  // Set an arbitrary qdot.
  VectorX<double> qdot(7);
  qdot << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0;

  // Calculate the angular and translational velocity associated with qdot.
  Vector6<double> v;
  mobilizer_->MapQDotToVelocity(*context_, qdot, &v);

  // Compute Nplus.
  MatrixX<double> Nplus(6, 7);
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);

  // Ensure N⁺(q) is used in `v = N⁺(q)⋅q̇`
  EXPECT_TRUE(CompareMatrices(v, Nplus * qdot, kTolerance,
                              MatrixCompareType::relative));

  // Notice that the rotational part of the arbitrary qdot violates a constraint
  // on the rotational unit quaternion q_WB.  Why?  Since (q_WB)ᵀ * q_WB = 1,
  // (q̇_WB)ᵀ * q_WB + (q_WB)ᵀ * q̇_WB = 2 * (q_WB)ᵀ * q̇_WB = 0.  However, since
  // the qdot above is arbitrary, this is not true.
  const Vector4<double> qdot_WB(qdot[0], qdot[1], qdot[2], qdot[3]);
  const double is_zero_if_proper_qdot =
      Vector4<double>(q_WB.w(), q_WB.x(), q_WB.y(), q_WB.z()).dot(qdot_WB);
  EXPECT_FALSE(std::abs(is_zero_if_proper_qdot) < 0.1);
}

TEST_F(QuaternionFloatingMobilizerTest, MapVelocityToQDotAndViceVersa) {
  // Set an arbitrary non-zero state, but with a non-unit quaternion.
  // Test with both a unnormalized and normalized quaternion q_FM.
  const Quaternion<double> q_nonUnit(0.7, 0.8, 0.9, -1.2);   // Unnormalized.
  const Quaternion<double> q_unit = q_nonUnit.normalized();  // Unit quaternion.
  const Vector3<double> wxyz(5.4, -9.8, 3.2);
  const Vector3<double> vxyz(0.2, 0.5, -0.87);
  const Vector6<double> v(wxyz[0], wxyz[1], wxyz[2], vxyz[0], vxyz[1], vxyz[2]);
  mobilizer_->SetQuaternion(context_.get(), q_unit);
  mobilizer_->SetAngularVelocity(context_.get(), wxyz);
  mobilizer_->SetTranslationalVelocity(context_.get(), vxyz);

  // Calculate the qdot associated with angular and translational velocity.
  Eigen::Matrix<double, 7, 1> qdot_unit;  // Calculate qdot using q_unit.
  mobilizer_->MapVelocityToQDot(*context_, v, &qdot_unit);

  // Use the previously calculated qdot to calculate an associated angular and
  // translational velocity -- and check that they match the original v.
  Vector6<double> v_from_qdot_unit;
  mobilizer_->MapQDotToVelocity(*context_, qdot_unit, &v_from_qdot_unit);
  EXPECT_TRUE(CompareMatrices(v, v_from_qdot_unit, 16 * kTolerance,
                              MatrixCompareType::relative));

  // Ensure that the calculated q̇ᵣ (rotational part of the calculated qdot)
  // satisfies the time-derivative of the quaternion constraint.
  // Mathematically: (qᵣ)ᵀ * qᵣ = constant, so it should be true that
  // (q̇ᵣ)ᵀ * qᵣ + (qᵣ)ᵀ * q̇ᵣ = 2 * (qᵣ)ᵀ * q̇ᵣ = 0. Due to the fact that q̇ᵣ is
  // calculated as q̇ᵣ = Nᵣ * vᵣ, we can prove this is true. Double check here.
  const Vector4<double> qrdot_unit(qdot_unit[0], qdot_unit[1], qdot_unit[2],
                                   qdot_unit[3]);
  double is_zero_if_OK_qdot =
      math::CalculateQuaternionDtConstraintViolation(q_unit, qrdot_unit);
  EXPECT_TRUE(std::abs(is_zero_if_OK_qdot) < 16 * kTolerance);

  // Repeat the previous calculations with a non-unit quaternion.
  mobilizer_->SetQuaternion(context_.get(), q_nonUnit);
  Eigen::Matrix<double, 7, 1> qdot_nonUnit;  // Calculate qdot using q_nonUnit.
  mobilizer_->MapVelocityToQDot(*context_, v, &qdot_nonUnit);
  Vector6<double> v_from_qdot_nonUnit;
  mobilizer_->MapQDotToVelocity(*context_, qdot_nonUnit, &v_from_qdot_nonUnit);
  EXPECT_TRUE(CompareMatrices(v, v_from_qdot_nonUnit, 16 * kTolerance,
                              MatrixCompareType::relative));
  const Vector4<double> qrdot_nonUnit(qdot_nonUnit[0], qdot_nonUnit[1],
                                      qdot_nonUnit[2], qdot_nonUnit[3]);
  is_zero_if_OK_qdot =
      math::CalculateQuaternionDtConstraintViolation(q_unit, qrdot_nonUnit);
  EXPECT_TRUE(std::abs(is_zero_if_OK_qdot) < 16 * kTolerance);

  // Verify qrdot_nonUnit = |qr_nonUnit| * qrdot_unit.
  const double s = q_nonUnit.norm();
  EXPECT_TRUE(CompareMatrices(qrdot_nonUnit, s * qrdot_unit, 16 * kTolerance,
                              MatrixCompareType::relative));

  // Verify vr_from_qdot_nonUnit = vr_from_qdot_unit.
  const Vector3<double> vr_from_qdot_nonUnit = v_from_qdot_nonUnit.head(3);
  const Vector3<double> vr_from_qdot_unit = v_from_qdot_unit.head(3);
  EXPECT_TRUE(CompareMatrices(vr_from_qdot_nonUnit, vr_from_qdot_unit,
                              16 * kTolerance, MatrixCompareType::relative));
}

TEST_F(QuaternionFloatingMobilizerTest, MapAccelerationToQDDotAndViceVersa) {
  // Set an arbitrary non-zero state.
  const math::RollPitchYaw<double> rpy(M_PI / 3, -M_PI / 4, M_PI / 5);
  Quaternion<double> q_FM = rpy.ToQuaternion();
  const Vector3<double> wxyz(5.4, -9.8, 3.2);
  const Vector3<double> vxyz(0.2, 0.5, -0.87);
  const Vector6<double> v(wxyz[0], wxyz[1], wxyz[2], vxyz[0], vxyz[1], vxyz[2]);
  mobilizer_->SetQuaternion(context_.get(), q_FM);
  mobilizer_->SetAngularVelocity(context_.get(), wxyz);
  mobilizer_->SetTranslationalVelocity(context_.get(), vxyz);

  // Set an arbitrary v̇ and use MapAccelerationToQDDot() to calculate q̈.
  // Reminder: v̇ = [ẇx, ẇy, ẇz, v̇x, v̇y, v̇z]ᵀ
  const Vector6<double> vdot(0.3, -0.2, 0.9, -1.2, 3.1, -2.3);
  Eigen::Matrix<double, 7, 1> qddot;
  mobilizer_->MapAccelerationToQDDot(*context_, vdot, &qddot);

  // Starting with the previous q̈, use MapQDDotToAcceleration() to calculate v̇.
  // Verify MapQDDotToAcceleration() is the inverse of MapAccelerationToQDDot().
  Vector6<double> vdot_redo;
  mobilizer_->MapQDDotToAcceleration(*context_, qddot, &vdot_redo);
  EXPECT_TRUE(CompareMatrices(vdot_redo, vdot, 16 * kTolerance,
                              MatrixCompareType::relative));

  // Compute the 7x6 N(q) matrix and its time-derivative Ṅ(q,q̇) that appear in
  // q̈ = Ṅ(q,q̇)⋅v + N(q)⋅v̇
  MatrixX<double> N(7, 6), Ndot(7, 6);
  mobilizer_->CalcNMatrix(*context_, &N);
  mobilizer_->CalcNDotMatrix(*context_, &Ndot);

  // Verify equivalence of q̈ = Ṅ(q,q̇)⋅v + N(q)⋅v̇ and MapAccelerationToQDDot().
  const Eigen::Matrix<double, 7, 1> qddot_expected = Ndot * v + N * vdot;
  EXPECT_TRUE(CompareMatrices(qddot, qddot_expected, kTolerance,
                              MatrixCompareType::relative));

  // Compute the 6x7 N⁺(q) matrix and its time-derivative Ṅ⁺(q,q̇) that appear in
  // v̇ = Ṅ⁺(q,q̇)⋅q̇ + N⁺(q)⋅q̈
  MatrixX<double> Nplus(6, 7), Nplusdot(6, 7);
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);
  mobilizer_->CalcNplusDotMatrix(*context_, &Nplusdot);

  // Verify equivalence of v̇ = Ṅ⁺(q,q̇)⋅q̇ + N⁺(q)⋅q̈ and MapQDDotToAcceleration().
  Eigen::Matrix<double, 7, 1> qdot;
  mobilizer_->MapVelocityToQDot(*context_, v, &qdot);
  const Vector6<double> vdot_expected = Nplusdot * qdot + Nplus * qddot;
  EXPECT_TRUE(CompareMatrices(vdot_expected, vdot, 16 * kTolerance,
                              MatrixCompareType::relative));

  // Ensure the N⁺(q) matrix is the left pseudoinverse of the N(q) matrix.
  // In other words, ensure Nplus * N = [I₆₆] (6x6 identity matrix).
  MatrixX<double> Nplus_x_N = Nplus * N;
  EXPECT_TRUE(CompareMatrices(Nplus_x_N, MatrixX<double>::Identity(6, 6),
                              kTolerance, MatrixCompareType::relative));

  // Ensure the rotation (upper-left block) part of the N(q) matrix is
  // 0.25 times the transpose of the rotation part of the N⁺(q) matrix.
  MatrixX<double> N_rotational = N.block<4, 3>(0, 0);
  MatrixX<double> Nplus_rotational = Nplus.block<3, 4>(0, 0);
  EXPECT_TRUE(CompareMatrices(N_rotational.transpose(), 0.25 * Nplus_rotational,
                              kTolerance, MatrixCompareType::relative));

  // Repeat these experiments with an unnormalized (non-unit) quaternion.
  // Use MapAccelerationToQDDot() to calculate q̈ and then use this calculated q̈
  // with MapQDDotToAcceleration() to calculate v̇.
  // Verify MapQDDotToAcceleration() is the inverse of MapAccelerationToQDDot().
  q_FM = Quaternion<double>(0.7, 0.8, 0.9, -1.2);  // Unnormalized.
  mobilizer_->SetQuaternion(context_.get(), q_FM);
  mobilizer_->MapAccelerationToQDDot(*context_, vdot, &qddot);
  mobilizer_->MapQDDotToAcceleration(*context_, qddot, &vdot_redo);
  EXPECT_TRUE(CompareMatrices(vdot_redo, vdot, 16 * kTolerance,
                              MatrixCompareType::relative));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
