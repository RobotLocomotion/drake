#include "drake/multibody/tree/rpy_floating_mobilizer.h"

#include <limits>
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/rpy_floating_joint.h"
#include "drake/multibody/tree/test/mobilizer_tester.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using math::RollPitchYawd;
using math::RotationMatrixd;
using Quaterniond = Eigen::Quaternion<double>;

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();

// Fixture to setup a simple model containing a space xyz floating mobilizer.
class RpyFloatingMobilizerTest : public MobilizerTester {
 public:
  // Creates a simple model consisting of a single body with a space xyz
  // floating mobilizer connecting it to the world.
  void SetUp() override {
    mobilizer_ = &AddJointAndFinalize<RpyFloatingJoint, RpyFloatingMobilizer>(
        std::make_unique<RpyFloatingJoint<double>>(
            "joint0", tree().world_body().body_frame(), body_->body_frame()));
    mutable_mobilizer_ = const_cast<RpyFloatingMobilizer<double>*>(mobilizer_);
  }

  // Helper to set the this fixture's context to an arbitrary non-zero state
  // comprised of arbitrary_rpy() and arbitrary_translation().
  void SetArbitraryNonZeroState() {
    mobilizer_->SetAngles(context_.get(), arbitrary_rpy().vector());
    mobilizer_->SetTranslation(context_.get(), arbitrary_translation());
  }

  RollPitchYawd arbitrary_rpy() const {
    return RollPitchYawd(M_PI / 3, -M_PI / 3, M_PI / 5);
  }
  Vector3d arbitrary_translation() const { return Vector3d(1.0, 2.0, 3.0); }

 protected:
  const RpyFloatingMobilizer<double>* mobilizer_{nullptr};
  RpyFloatingMobilizer<double>* mutable_mobilizer_{nullptr};
};

TEST_F(RpyFloatingMobilizerTest, CanRotateOrTranslate) {
  EXPECT_TRUE(mobilizer_->can_rotate());
  EXPECT_TRUE(mobilizer_->can_translate());
}

// Verifies methods to mutate and access the context.
TEST_F(RpyFloatingMobilizerTest, BasicIntrospection) {
  EXPECT_TRUE(mobilizer_->has_six_dofs());
  EXPECT_FALSE(mobilizer_->has_quaternion_dofs());
}

TEST_F(RpyFloatingMobilizerTest, NameSuffix) {
  EXPECT_EQ(mobilizer_->position_suffix(0), "qx");
  EXPECT_EQ(mobilizer_->position_suffix(1), "qy");
  EXPECT_EQ(mobilizer_->position_suffix(2), "qz");
  EXPECT_EQ(mobilizer_->position_suffix(3), "x");
  EXPECT_EQ(mobilizer_->position_suffix(4), "y");
  EXPECT_EQ(mobilizer_->position_suffix(5), "z");
  EXPECT_EQ(mobilizer_->velocity_suffix(0), "wx");
  EXPECT_EQ(mobilizer_->velocity_suffix(1), "wy");
  EXPECT_EQ(mobilizer_->velocity_suffix(2), "wz");
  EXPECT_EQ(mobilizer_->velocity_suffix(3), "vx");
  EXPECT_EQ(mobilizer_->velocity_suffix(4), "vy");
  EXPECT_EQ(mobilizer_->velocity_suffix(5), "vz");
}

// Verifies methods to mutate and access the context.
TEST_F(RpyFloatingMobilizerTest, ConfigurationAccessAndMutation) {
  SetArbitraryNonZeroState();
  EXPECT_EQ(mobilizer_->get_angles(*context_), arbitrary_rpy().vector());
  EXPECT_EQ(mobilizer_->get_translation(*context_), arbitrary_translation());
  Vector6<double> q;
  q << arbitrary_rpy().vector(), arbitrary_translation();
  EXPECT_EQ(mobilizer_->get_generalized_positions(*context_), q);
}

TEST_F(RpyFloatingMobilizerTest, SetFromRigidTransform) {
  SetArbitraryNonZeroState();
  const RigidTransformd X_WB(arbitrary_rpy(), arbitrary_translation());
  mobilizer_->SetFromRigidTransform(context_.get(), X_WB);
  EXPECT_TRUE(CompareMatrices(mobilizer_->get_angles(*context_),
                              arbitrary_rpy().vector(), kTolerance,
                              MatrixCompareType::relative));
  EXPECT_EQ(mobilizer_->get_translation(*context_), arbitrary_translation());
}

TEST_F(RpyFloatingMobilizerTest, VelocityAccessAndMutation) {
  const Vector3d w_FM(M_PI / 3, -M_PI / 3, M_PI / 5);
  mobilizer_->SetAngularVelocity(context_.get(), w_FM);
  EXPECT_EQ(mobilizer_->get_angular_velocity(*context_), w_FM);

  const Vector3d v_FM(1.0, 2.0, 3.0);
  mobilizer_->SetTranslationalVelocity(context_.get(), v_FM);
  EXPECT_EQ(mobilizer_->get_translational_velocity(*context_), v_FM);

  const auto v = (Vector6<double>() << w_FM, v_FM).finished();
  EXPECT_EQ(mobilizer_->get_generalized_velocities(*context_), v);
}

TEST_F(RpyFloatingMobilizerTest, ZeroState) {
  SetArbitraryNonZeroState();

  // The non-zero state is not the identity transform as expected.
  EXPECT_FALSE(
      mobilizer_->CalcAcrossMobilizerTransform(*context_).IsExactlyIdentity());

  // Set the "zero state" for this mobilizer, which does happen to be that of
  // an identity rigid transform.
  mobilizer_->SetZeroState(*context_, &context_->get_mutable_state());
  EXPECT_TRUE(
      mobilizer_->CalcAcrossMobilizerTransform(*context_).IsExactlyIdentity());
}

TEST_F(RpyFloatingMobilizerTest, CalcAcrossMobilizerTransform) {
  const double kTol = 4 * std::numeric_limits<double>::epsilon();
  const Vector3d rpy_value(M_PI / 3, -M_PI / 3, M_PI / 5);
  const Vector3d translation(1.0, 2.0, 3.0);
  mobilizer_->SetAngles(context_.get(), rpy_value);
  mobilizer_->SetTranslation(context_.get(), translation);
  const double* q =
      &context_
           ->get_continuous_state_vector()[mobilizer_->position_start_in_q()];
  RigidTransformd X_FM(mobilizer_->CalcAcrossMobilizerTransform(*context_));

  const RigidTransformd X_FM_expected(RollPitchYawd(rpy_value), translation);
  EXPECT_TRUE(X_FM.IsNearlyEqualTo(X_FM_expected, kTol));

  // Now check the fast inline methods.
  RigidTransformd fast_X_FM = mobilizer_->calc_X_FM(q);
  EXPECT_TRUE(fast_X_FM.IsNearlyEqualTo(X_FM, kTol));
  const Vector3d new_translation(1.5, 2.5, 3.5);
  const Vector3d new_rpy_value(M_PI / 4, -M_PI / 4, M_PI / 7);
  mobilizer_->SetAngles(context_.get(), new_rpy_value);
  mobilizer_->SetTranslation(context_.get(), new_translation);
  X_FM = mobilizer_->CalcAcrossMobilizerTransform(*context_);
  mobilizer_->update_X_FM(q, &fast_X_FM);
  EXPECT_TRUE(fast_X_FM.IsNearlyEqualTo(X_FM, kTol));

  TestApplyR_FM(X_FM, *mobilizer_);
  TestPrePostMultiplyByX_FM(X_FM, *mobilizer_);
}

TEST_F(RpyFloatingMobilizerTest, SetGetPosePair) {
  const Quaterniond set_quaternion(RollPitchYawd(0.1, 0.2, 0.3).ToQuaternion());
  const Vector3d set_translation(1.0, 2.0, 3.0);
  const RigidTransformd set_pose(set_quaternion, set_translation);

  SetArbitraryNonZeroState();
  const std::pair<Eigen::Quaternion<double>, Vector3d> before =
      mobilizer_->GetPosePair(*context_);

  EXPECT_FALSE(math::RigidTransform(before.first, before.second)
                   .IsNearlyEqualTo(set_pose, 1e-8));

  mobilizer_->SetPosePair(*context_, set_quaternion, set_translation,
                          &context_->get_mutable_state());

  const std::pair<Quaterniond, Vector3d> after =
      mobilizer_->GetPosePair(*context_);

  EXPECT_TRUE(math::RigidTransform(after.first, after.second)
                  .IsNearlyEqualTo(set_pose, 1e-14));
}

TEST_F(RpyFloatingMobilizerTest, SetGetSpatialVelocity) {
  const SpatialVelocity<double> set_V(Vector3d(1.0, 2.0, 3.0),
                                      Vector3d(4.0, 5.0, 6.0));

  SetArbitraryNonZeroState();
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

TEST_F(RpyFloatingMobilizerTest, RandomState) {
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

  Eigen::Matrix<symbolic::Expression, 3, 1> angles_distribution;
  Eigen::Matrix<symbolic::Expression, 3, 1> translation_distribution;
  for (int i = 0; i < 3; i++) {
    angles_distribution[i] = (0.0125 * uniform(generator) + i + 1.0);
    translation_distribution[i] = uniform(generator) + i + 1.0;
  }

  // Set position to be random, but not velocity (yet).
  mutable_mobilizer_->set_random_angles_distribution(angles_distribution);
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

// Verify the correctness of across-mobilizer quantities; X_FM, V_FM, A_FM.
TEST_F(RpyFloatingMobilizerTest, CalcAcrossMobilizer) {
  SetArbitraryNonZeroState();

  const RigidTransformd X_FM_expected(arbitrary_rpy(), arbitrary_translation());
  const RigidTransformd X_FM =
      mobilizer_->CalcAcrossMobilizerTransform(*context_);
  EXPECT_TRUE(CompareMatrices(X_FM.GetAsMatrix34(),
                              X_FM_expected.GetAsMatrix34(), kTolerance,
                              MatrixCompareType::relative));

  // Verify across mobilizer spatial velocity for an arbitrary non-zero state.
  Vector6<double> v;
  v << 1, 2, 3, 4, 5, 6;
  const SpatialVelocity<double> V_FM =
      mobilizer_->CalcAcrossMobilizerSpatialVelocity(*context_, v);
  EXPECT_TRUE(CompareMatrices(V_FM.get_coeffs(), v, kTolerance,
                              MatrixCompareType::relative));

  // Verify across mobilizer spatial velocity for an arbitrary non-zero state
  // and generalized accelrations.
  const SpatialAcceleration<double> A_FM =
      mobilizer_->CalcAcrossMobilizerSpatialAcceleration(*context_, v);
  EXPECT_TRUE(CompareMatrices(A_FM.get_coeffs(), v, kTolerance,
                              MatrixCompareType::relative));

  // Force projection across the mobilizer. Since the generalized velocities for
  // this mobilizer correspond to the spatial velocity across the mobilizer, the
  // generalized forces must equal the spatial force on the mobilizer frame due
  // to the principle of virtual work.
  SpatialForce<double> F(v);
  Vector6<double> tau;
  mobilizer_->ProjectSpatialForce(*context_, F, tau);
  EXPECT_TRUE(CompareMatrices(F.get_coeffs(), tau, kTolerance,
                              MatrixCompareType::relative));
}

TEST_F(RpyFloatingMobilizerTest, MapVelocityToQdotAndBack) {
  EXPECT_FALSE(mobilizer_->is_velocity_equal_to_qdot());

  SetArbitraryNonZeroState();
  const Vector6<double> v = (Vector6<double>() << 1, 2, 3, 4, 5, 6).finished();
  Vector6<double> qdot;
  mobilizer_->MapVelocityToQDot(*context_, v, &qdot);
  Vector6<double> v_back;
  mobilizer_->MapQDotToVelocity(*context_, qdot, &v_back);
  EXPECT_TRUE(
      CompareMatrices(v, v_back, kTolerance, MatrixCompareType::relative));
}

// For an arbitrary state verify that the computed Nplus(q) matrix is the
// inverse of N(q).
TEST_F(RpyFloatingMobilizerTest, KinematicMapping) {
  RollPitchYawd rpy(M_PI / 3, -M_PI / 3, M_PI / 5);
  mobilizer_->SetAngles(context_.get(), rpy.vector());

  ASSERT_EQ(mobilizer_->num_positions(), 6);
  ASSERT_EQ(mobilizer_->num_velocities(), 6);

  // Compute N.
  Matrix6<double> N;
  mobilizer_->CalcNMatrix(*context_, &N);

  // Compute Nplus.
  Matrix6<double> Nplus;
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);

  // Verify that Nplus is the inverse of N.
  MatrixX<double> N_x_Nplus = N * Nplus;
  MatrixX<double> Nplus_x_N = Nplus * N;

  EXPECT_TRUE(CompareMatrices(N_x_Nplus, Matrix6<double>::Identity(),
                              kTolerance, MatrixCompareType::relative));
  EXPECT_TRUE(CompareMatrices(Nplus_x_N, Matrix6<double>::Identity(),
                              kTolerance, MatrixCompareType::relative));
}

TEST_F(RpyFloatingMobilizerTest, MapUsesN) {
  SetArbitraryNonZeroState();
  const Vector6<double> v = (Vector6<double>() << 1, 2, 3, 4, 5, 6).finished();
  Vector6<double> qdot;
  mobilizer_->MapVelocityToQDot(*context_, v, &qdot);

  // Compute N.
  Matrix6<double> N;
  mobilizer_->CalcNMatrix(*context_, &N);

  // Ensure N(q) is used in q̇ = N(q)⋅v
  EXPECT_TRUE(
      CompareMatrices(qdot, N * v, kTolerance, MatrixCompareType::relative));
}

TEST_F(RpyFloatingMobilizerTest, MapUsesNplus) {
  SetArbitraryNonZeroState();
  const Vector6<double> qdot =
      (Vector6<double>() << 1, 2, 3, 4, 5, 6).finished();
  Vector6<double> v;
  mobilizer_->MapQDotToVelocity(*context_, qdot, &v);

  // Compute Nplus.
  Matrix6<double> Nplus;
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);

  // Ensure N⁺(q) is used in v = N⁺(q)⋅q̇
  EXPECT_TRUE(CompareMatrices(v, Nplus * qdot, kTolerance,
                              MatrixCompareType::relative));
}

TEST_F(RpyFloatingMobilizerTest, SingularityError) {
  // Set state in singularity
  const Vector3d rpy_value(M_PI / 3, M_PI / 2, M_PI / 5);
  mobilizer_->SetAngles(context_.get(), rpy_value);

  // Set arbitrary qdot and MapVelocityToQDot.
  const Vector6<double> v = (Vector6<double>() << 1, 2, 3, 4, 5, 6).finished();
  Vector6<double> qdot;
  DRAKE_EXPECT_THROWS_MESSAGE(
      mobilizer_->MapVelocityToQDot(*context_, v, &qdot), ".*singularity.*");

  // Compute N.
  MatrixX<double> N(6, 6);
  DRAKE_EXPECT_THROWS_MESSAGE(mobilizer_->CalcNMatrix(*context_, &N),
                              ".*singularity.*");
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
