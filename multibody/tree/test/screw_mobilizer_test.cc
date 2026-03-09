#include "drake/multibody/tree/screw_mobilizer.h"

#include <exception>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/screw_joint.h"
#include "drake/multibody/tree/test/mobilizer_tester.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Matrix1d = Eigen::Matrix<double, 1, 1>;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransformd;
using math::RotationMatrixd;

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();
constexpr double kScrewPitch = 1.e-2;
const Vector3<double> kScrewAxis(0.36, -0.48, 0.8);

// Fixture to setup a simple MBT model containing a screw mobilizer.
class ScrewMobilizerTest : public MobilizerTester {
 public:
  // Creates a simple model consisting of a single body with a screw
  // mobilizer connecting it to the world.
  void SetUp() override {
    mobilizer_ = &AddJointAndFinalize<ScrewJoint, ScrewMobilizer>(
        std::make_unique<ScrewJoint<double>>(
            "joint0", tree().world_body().body_frame(), body_->body_frame(),
            kScrewAxis, kScrewPitch, 0.0));
    mutable_mobilizer_ = const_cast<ScrewMobilizer<double>*>(mobilizer_);
  }

 protected:
  const ScrewMobilizer<double>* mobilizer_{nullptr};
  ScrewMobilizer<double>* mutable_mobilizer_{nullptr};
};

TEST_F(ScrewMobilizerTest, CanRotateOrTranslate) {
  EXPECT_TRUE(mobilizer_->can_rotate());
  EXPECT_TRUE(mobilizer_->can_translate());
}

TEST_F(ScrewMobilizerTest, ScrewPitchAccess) {
  EXPECT_EQ(mobilizer_->screw_pitch(), kScrewPitch);
}

TEST_F(ScrewMobilizerTest, ExceptionRaisingWhenZeroPitch) {
  const double zero_screw_pitch{0};
  const SpanningForest::Mobod dummy_mobod(MobodIndex(0), LinkOrdinal(0));
  ScrewMobilizer<double> zero_pitch_screw_mobilizer(
      dummy_mobod, tree().world_body().body_frame(), body_->body_frame(),
      Vector3<double>::UnitZ(), zero_screw_pitch);

  const double translation_z{1.0};
  const double velocity_z{2.0};
  EXPECT_THROW(
      zero_pitch_screw_mobilizer.SetTranslation(context_.get(), translation_z),
      std::exception);
  EXPECT_THROW(
      zero_pitch_screw_mobilizer.SetTranslationRate(context_.get(), velocity_z),
      std::exception);
}

TEST_F(ScrewMobilizerTest, StateAccess) {
  // Verify we can set a screw mobilizer configuration given the model's
  // context.
  const double translation_z_first{1.0};
  const double translation_z_second{2.0};
  const double angle_z_second{translation_z_second * 2 * M_PI / kScrewPitch};

  mobilizer_->SetTranslation(context_.get(), translation_z_first);
  EXPECT_EQ(mobilizer_->get_translation(*context_), translation_z_first);

  mobilizer_->SetTranslation(context_.get(), translation_z_second);
  EXPECT_EQ(mobilizer_->get_translation(*context_), translation_z_second);
  EXPECT_EQ(mobilizer_->get_angle(*context_), angle_z_second);

  const double angle_z_first{1.0 * 180.0 / M_PI};

  mobilizer_->SetAngle(context_.get(), angle_z_first);
  EXPECT_EQ(mobilizer_->get_angle(*context_), angle_z_first);

  mobilizer_->SetAngle(context_.get(), angle_z_second);
  EXPECT_EQ(mobilizer_->get_angle(*context_), angle_z_second);

  const double velocity_z_first{1.0};
  const double velocity_z_second{2.0};
  const double angular_velocity_z_second{velocity_z_second * 2 * M_PI /
                                         kScrewPitch};

  // Verify we can set a screw mobilizer velocities given the model's context.
  mobilizer_->SetTranslationRate(context_.get(), velocity_z_first);
  EXPECT_EQ(mobilizer_->get_translation_rate(*context_), velocity_z_first);

  mobilizer_->SetTranslationRate(context_.get(), velocity_z_second);
  EXPECT_EQ(mobilizer_->get_translation_rate(*context_), velocity_z_second);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), angular_velocity_z_second);

  const double angular_velocity_z_first{1.0 * 180.0 / M_PI};

  mobilizer_->SetAngularRate(context_.get(), angular_velocity_z_first);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), angular_velocity_z_first);

  mobilizer_->SetAngularRate(context_.get(), angular_velocity_z_second);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), angular_velocity_z_second);
}

TEST_F(ScrewMobilizerTest, ZeroState) {
  const double angle_z{1.0 * 180.0 / M_PI};
  const double angular_velocity_z{2.0 * 180.0 / M_PI};

  const double translation_z{angle_z * kScrewPitch / (2.0 * M_PI)};
  const double velocity_z{angular_velocity_z * kScrewPitch / (2.0 * M_PI)};

  // Set the state to some arbitrary non-zero value.
  mobilizer_->SetAngle(context_.get(), angle_z);
  mobilizer_->SetAngularRate(context_.get(), angular_velocity_z);
  EXPECT_EQ(mobilizer_->get_angle(*context_), angle_z);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), angular_velocity_z);
  EXPECT_LE(std::fabs(mobilizer_->get_translation(*context_) - translation_z),
            kTolerance);
  EXPECT_LE(std::fabs(mobilizer_->get_translation_rate(*context_) - velocity_z),
            kTolerance);

  // Set the mobilizer state to zero.
  mobilizer_->SetZeroState(*context_, &context_->get_mutable_state());
  EXPECT_EQ(mobilizer_->get_translation(*context_), 0.0);
  EXPECT_EQ(mobilizer_->get_translation_rate(*context_), 0.0);
  EXPECT_EQ(mobilizer_->get_angle(*context_), 0.0);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), 0.0);
}

TEST_F(ScrewMobilizerTest, DefaultPosition) {
  EXPECT_EQ(mobilizer_->get_translation(*context_), 0.0);
  EXPECT_EQ(mobilizer_->get_angle(*context_), 0.0);

  const Vector1d new_default(0.4);
  mutable_mobilizer_->set_default_position(new_default);
  mobilizer_->set_default_state(*context_, &context_->get_mutable_state());

  EXPECT_EQ(mobilizer_->get_angle(*context_), new_default(0));
  EXPECT_EQ(mobilizer_->get_translation(*context_),
            new_default(0) * kScrewPitch / (2.0 * M_PI));
}

TEST_F(ScrewMobilizerTest, RandomState) {
  RandomGenerator generator;
  std::uniform_real_distribution<symbolic::Expression> uniform;

  // Default behavior is to set to zero.
  mobilizer_->set_random_state(*context_, &context_->get_mutable_state(),
                               &generator);
  EXPECT_EQ(mobilizer_->get_angle(*context_), 0.0);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), 0.0);

  // Set position to be random, but not velocity (yet).
  mutable_mobilizer_->set_random_position_distribution(
      Vector1<symbolic::Expression>(uniform(generator) + 1.0));
  mobilizer_->set_random_state(*context_, &context_->get_mutable_state(),
                               &generator);
  EXPECT_GE(mobilizer_->get_angle(*context_), 1.0);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), 0.0);

  // Set the velocity distribution.  Now both should be random.
  mutable_mobilizer_->set_random_velocity_distribution(
      Vector1<symbolic::Expression>(uniform(generator) - 1.0));
  mobilizer_->set_random_state(*context_, &context_->get_mutable_state(),
                               &generator);
  EXPECT_GE(mobilizer_->get_angle(*context_), 1.0);
  EXPECT_GE(mobilizer_->get_angular_rate(*context_), -1.0);

  // Check that they change on a second draw from the distribution.
  const double last_translation = mobilizer_->get_translation(*context_);
  const double last_translational_rate =
      mobilizer_->get_translation_rate(*context_);
  const double last_angle = mobilizer_->get_angle(*context_);
  const double last_angular_rate = mobilizer_->get_angular_rate(*context_);
  mobilizer_->set_random_state(*context_, &context_->get_mutable_state(),
                               &generator);
  EXPECT_NE(mobilizer_->get_translation(*context_), last_translation);
  EXPECT_NE(mobilizer_->get_translation_rate(*context_),
            last_translational_rate);
  EXPECT_NE(mobilizer_->get_angle(*context_), last_angle);
  EXPECT_NE(mobilizer_->get_angular_rate(*context_), last_angular_rate);
}

TEST_F(ScrewMobilizerTest, CalcAcrossMobilizerTransform) {
  const double kTol = 4 * std::numeric_limits<double>::epsilon();
  const double angle = 1.5;
  mobilizer_->SetAngle(context_.get(), angle);
  RigidTransformd X_FM(mobilizer_->CalcAcrossMobilizerTransform(*context_));

  Vector3d X_FM_translation;
  X_FM_translation << kScrewAxis * angle / (2 * M_PI) * kScrewPitch;
  const RigidTransformd X_FM_expected(
      Eigen::AngleAxis<double>(angle, kScrewAxis), X_FM_translation);

  EXPECT_TRUE(CompareMatrices(X_FM.GetAsMatrix34(),
                              X_FM_expected.GetAsMatrix34(), kTolerance,
                              MatrixCompareType::relative));

  // Now check the fast inline methods.
  RigidTransformd fast_X_FM = mobilizer_->calc_X_FM(&angle);
  EXPECT_TRUE(fast_X_FM.IsNearlyEqualTo(X_FM, kTol));
  const double new_angle = 2.0;
  mobilizer_->SetAngle(context_.get(), new_angle);
  X_FM = mobilizer_->CalcAcrossMobilizerTransform(*context_);
  mobilizer_->update_X_FM(&new_angle, &fast_X_FM);
  EXPECT_TRUE(fast_X_FM.IsNearlyEqualTo(X_FM, kTol));

  TestApplyR_FM(X_FM, *mobilizer_);
  TestPrePostMultiplyByX_FM(X_FM, *mobilizer_);
}

TEST_F(ScrewMobilizerTest, CalcAcrossMobilizerSpatialVeloctiy) {
  const double angle = 1.5;
  const Vector1d angular_velocity(0.1);
  mobilizer_->SetAngle(context_.get(), angle);
  const SpatialVelocity<double> V_FM =
      mobilizer_->CalcAcrossMobilizerSpatialVelocity(*context_,
                                                     angular_velocity);

  VectorXd v_expected(6);
  v_expected << kScrewAxis * angular_velocity[0],
      kScrewAxis * angular_velocity[0] / (2 * M_PI) * kScrewPitch;
  const SpatialVelocity<double> V_FM_expected(v_expected);

  EXPECT_TRUE(V_FM.IsApprox(V_FM_expected, kTolerance));
}

TEST_F(ScrewMobilizerTest, CalcAcrossMobilizerSpatialAcceleration) {
  const double angle = 1.5;
  const double angle_rate = 3;
  const Vector1d angle_acceleration(4.5);
  mobilizer_->SetAngle(context_.get(), angle);
  mobilizer_->SetAngularRate(context_.get(), angle_rate);

  const SpatialAcceleration<double> A_FM =
      mobilizer_->CalcAcrossMobilizerSpatialAcceleration(*context_,
                                                         angle_acceleration);

  VectorXd a_expected(6);
  a_expected << kScrewAxis * angle_acceleration[0],
      kScrewAxis * angle_acceleration[0] / (2 * M_PI) * kScrewPitch;
  const SpatialAcceleration<double> A_FM_expected(a_expected);

  EXPECT_TRUE(A_FM.IsApprox(A_FM_expected, kTolerance));
}

TEST_F(ScrewMobilizerTest, ProjectSpatialForce) {
  const double translation(0.5);
  const double angle = 1.5;
  mobilizer_->SetTranslation(context_.get(), translation);
  mobilizer_->SetAngle(context_.get(), angle);

  const Vector3d torque_Mo_F(1.0, 2.0, 3.0);
  const Vector3d force_Mo_F(1.0, 2.0, 3.0);
  const SpatialForce<double> F_Mo_F(torque_Mo_F, force_Mo_F);
  Vector1d tau;
  mobilizer_->ProjectSpatialForce(*context_, F_Mo_F, tau);

  const Vector1d tau_expected(torque_Mo_F.dot(kScrewAxis) +
                              force_Mo_F.dot(kScrewAxis) / (2 * M_PI) *
                                  kScrewPitch);
  EXPECT_TRUE(CompareMatrices(tau, tau_expected, kTolerance,
                              MatrixCompareType::relative));

  // Power across the joint computed from spatial quantities should exactly
  // match the result computed from generalized velocities and forces
  // V_FM.dot(F_Mo) == tau * v
  const Vector1d angular_velocity(0.1);
  const SpatialVelocity<double> V_FM =
      mobilizer_->CalcAcrossMobilizerSpatialVelocity(*context_,
                                                     angular_velocity);
  EXPECT_NEAR(V_FM.dot(F_Mo_F), tau[0] * angular_velocity[0], kTolerance);
}

TEST_F(ScrewMobilizerTest, MapVelocityToQDotAndBack) {
  EXPECT_TRUE(mobilizer_->is_velocity_equal_to_qdot());

  Vector1d v(1.5);
  Vector1d qdot;
  mobilizer_->MapVelocityToQDot(*context_, v, &qdot);
  EXPECT_TRUE(
      CompareMatrices(qdot, v, kTolerance, MatrixCompareType::relative));

  qdot(0) = -std::sqrt(2);
  mobilizer_->MapQDotToVelocity(*context_, qdot, &v);
  EXPECT_TRUE(
      CompareMatrices(v, qdot, kTolerance, MatrixCompareType::relative));

  // Test relationship between q̈ (2ⁿᵈ time derivatives of generalized positions)
  // and v̇ (1ˢᵗ time derivatives of generalized velocities) and vice-versa.
  Vector1d vdot(1.2345);
  Vector1d qddot;
  mobilizer_->MapAccelerationToQDDot(*context_, vdot, &qddot);
  EXPECT_NEAR(qddot(0), vdot(0), kTolerance);

  qddot(0) = -std::sqrt(5);
  mobilizer_->MapQDDotToAcceleration(*context_, qddot, &vdot);
  EXPECT_NEAR(vdot(0), qddot(0), kTolerance);
}

TEST_F(ScrewMobilizerTest, KinematicMapping) {
  // For this joint, Nplus = I independently of the state. We therefore set the
  // state to NaN in order to verify this.
  tree()
      .GetMutablePositionsAndVelocities(context_.get())
      .setConstant(std::numeric_limits<double>::quiet_NaN());

  // Compute N.
  MatrixX<double> N(1, 1);
  mobilizer_->CalcNMatrix(*context_, &N);
  EXPECT_EQ(N, Matrix1d::Identity().eval());

  // Compute Nplus.
  MatrixX<double> Nplus(1, 1);
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);
  EXPECT_EQ(Nplus, Matrix1d::Identity().eval());

  // Ensure Ṅ(q,q̇) = 1x1 zero matrix.
  MatrixX<double> NDot(1, 1);
  mobilizer_->CalcNDotMatrix(*context_, &NDot);
  EXPECT_EQ(NDot(0, 0), 0.0);

  // Ensure Ṅ⁺(q,q̇) = 1x1 zero matrix.
  MatrixX<double> NplusDot(1, 1);
  mobilizer_->CalcNplusDotMatrix(*context_, &NplusDot);
  EXPECT_EQ(NplusDot(0, 0), 0.0);
}

TEST_F(ScrewMobilizerTest, MapUsesN) {
  // Set an arbitrary "non-zero" state.
  const double angle_z{1.0 * 180.0 / M_PI};
  mobilizer_->SetAngle(context_.get(), angle_z);

  // Set arbitrary v and MapVelocityToQDot.
  Vector1d v(1.5);
  Vector1d qdot;
  mobilizer_->MapVelocityToQDot(*context_, v, &qdot);

  // Compute N.
  MatrixX<double> N(1, 1);
  mobilizer_->CalcNMatrix(*context_, &N);

  // Ensure N(q) is used in `q̇ = N(q)⋅v`
  EXPECT_EQ(qdot, N * v);
}

TEST_F(ScrewMobilizerTest, MapUsesNplus) {
  // Set an arbitrary "non-zero" state.
  const double angle_z{1.0 * 180.0 / M_PI};
  mobilizer_->SetAngle(context_.get(), angle_z);

  // Set arbitrary qdot and MapQDotToVelocity.
  Vector1d qdot(1.5);
  Vector1d v;
  mobilizer_->MapQDotToVelocity(*context_, qdot, &v);

  // Compute Nplus.
  MatrixX<double> Nplus(1, 1);
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);

  // Ensure N⁺(q) is used in `v = N⁺(q)⋅q̇`
  EXPECT_EQ(v, Nplus * qdot);
}
}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
