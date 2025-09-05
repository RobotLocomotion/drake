#include "drake/multibody/tree/revolute_mobilizer.h"

#include <limits>
#include <memory>

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/test/mobilizer_tester.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::AngleAxisd;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;
using std::make_unique;
using std::unique_ptr;
using systems::Context;

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();

// Fixture to setup a simple MBT model containing a revolute mobilizer.
class RevoluteMobilizerTest : public MobilizerTester {
 public:
  void SetUp() override {
    // The axis is not one of the coordinate axes, so we'll expect to get
    // new F & M frames that rotate around their common z axis.
    const RevoluteMobilizer<double>& const_mobilizer =
        AddJointAndFinalize<RevoluteJoint, RevoluteMobilizer>(
            std::make_unique<RevoluteJoint<double>>(
                "joint0", tree().world_body().body_frame(), body_->body_frame(),
                axis_Jp_));
    // Mobilizers are marked ephemeral only if the modeled joint is ephemeral
    // (that is, added automatically).
    mobilizer_ = const_cast<RevoluteMobilizer<double>*>(&const_mobilizer);
    EXPECT_FALSE(mobilizer_->is_ephemeral());
  }

 protected:
  RevoluteMobilizer<double>* mobilizer_{nullptr};
  const Vector3d axis_Jp_{1.0, 2.0, 3.0};  // also axis_Jc
};

TEST_F(RevoluteMobilizerTest, CanRotateOrTranslate) {
  EXPECT_TRUE(mobilizer_->can_rotate());
  EXPECT_FALSE(mobilizer_->can_translate());
}

// Even though we started with a _joint_ axis vector with arbitrary components,
// we expect that the mobilizer will have a unit vector axis, and in particular
// that we chose the z axis. We expect exact integer components, no roundoff.
TEST_F(RevoluteMobilizerTest, AxisIsNormalizedAtConstruction) {
  EXPECT_EQ(mobilizer_->revolute_axis(), Vector3d(0, 0, 1));
}

// Verifies method to mutate and access the context.
TEST_F(RevoluteMobilizerTest, StateAccess) {
  const double some_value1 = 1.5;
  const double some_value2 = std::sqrt(2);
  // Verify we can set a revolute mobilizer position given the model's context.
  mobilizer_->SetAngle(context_.get(), some_value1);
  EXPECT_EQ(mobilizer_->get_angle(*context_), some_value1);
  mobilizer_->SetAngle(context_.get(), some_value2);
  EXPECT_EQ(mobilizer_->get_angle(*context_), some_value2);

  // Verify we can set a revolute mobilizer position rate given the model's
  // context.
  mobilizer_->SetAngularRate(context_.get(), some_value1);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), some_value1);
  mobilizer_->SetAngularRate(context_.get(), some_value2);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), some_value2);
}

TEST_F(RevoluteMobilizerTest, ZeroState) {
  const double some_value1 = 1.5;
  const double some_value2 = std::sqrt(2);
  // Set the state to some arbitrary non-zero value.
  mobilizer_->SetAngle(context_.get(), some_value1);
  EXPECT_EQ(mobilizer_->get_angle(*context_), some_value1);
  mobilizer_->SetAngularRate(context_.get(), some_value2);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), some_value2);

  // Set the "zero state" for this mobilizer, which does happen to be that of
  // zero position and velocity.
  mobilizer_->SetZeroState(*context_, &context_->get_mutable_state());
  EXPECT_EQ(mobilizer_->get_angle(*context_), 0);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), 0);
}

TEST_F(RevoluteMobilizerTest, DefaultPosition) {
  EXPECT_EQ(mobilizer_->get_angle(*context_), 0);

  mobilizer_->set_default_position(Vector1d{.4});
  mobilizer_->set_default_state(*context_, &context_->get_mutable_state());

  EXPECT_EQ(mobilizer_->get_angle(*context_), .4);
}

TEST_F(RevoluteMobilizerTest, RandomState) {
  RandomGenerator generator;
  std::uniform_real_distribution<symbolic::Expression> uniform;

  // Default behavior is to set to zero.
  mobilizer_->set_random_state(*context_, &context_->get_mutable_state(),
                               &generator);
  EXPECT_EQ(mobilizer_->get_angle(*context_), 0);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), 0);

  // Set position to be random, but not velocity (yet).
  mobilizer_->set_random_position_distribution(
      Vector1<symbolic::Expression>(uniform(generator) + 2.0));
  mobilizer_->set_random_state(*context_, &context_->get_mutable_state(),
                               &generator);
  EXPECT_GE(mobilizer_->get_angle(*context_), 2.0);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), 0);

  // Set the velocity distribution.  Now both should be random.
  mobilizer_->set_random_velocity_distribution(
      Vector1<symbolic::Expression>(uniform(generator) - 2.0));
  mobilizer_->set_random_state(*context_, &context_->get_mutable_state(),
                               &generator);
  EXPECT_GE(mobilizer_->get_angle(*context_), 2.0);
  EXPECT_LE(mobilizer_->get_angular_rate(*context_), -1.0);

  // Check that they change on a second draw from the distribution.
  const double last_angle = mobilizer_->get_angle(*context_);
  const double last_angular_rate = mobilizer_->get_angular_rate(*context_);
  mobilizer_->set_random_state(*context_, &context_->get_mutable_state(),
                               &generator);
  EXPECT_NE(mobilizer_->get_angle(*context_), last_angle);
  EXPECT_NE(mobilizer_->get_angular_rate(*context_), last_angular_rate);
}

TEST_F(RevoluteMobilizerTest, CalcAcrossMobilizerTransform) {
  const double kTol = 4 * std::numeric_limits<double>::epsilon();
  const double angle = 1.5;
  mobilizer_->SetAngle(context_.get(), angle);
  RigidTransformd X_FM(mobilizer_->CalcAcrossMobilizerTransform(*context_));

  const RigidTransformd X_FM_expected(
      RotationMatrixd(AngleAxisd(angle, mobilizer_->revolute_axis())));

  // Though checked below, we make it explicit here that this mobilizer should
  // introduce no translations at all.
  EXPECT_EQ(X_FM.translation(), Vector3d::Zero());
  EXPECT_TRUE(CompareMatrices(X_FM.GetAsMatrix34(),
                              X_FM_expected.GetAsMatrix34(), kTolerance,
                              MatrixCompareType::relative));

  // Now check the fast inline methods. Since we used a general axis above,
  // this should have been modeled with a z-axial mobilizer.
  auto mobilizer_z =
      dynamic_cast<const RevoluteMobilizerAxial<double, 2>*>(mobilizer_);
  ASSERT_NE(mobilizer_z, nullptr);
  RigidTransformd fast_X_FM = mobilizer_z->calc_X_FM(&angle);
  EXPECT_TRUE(fast_X_FM.IsNearlyEqualTo(X_FM, kTol));
  const double new_angle = 2.0;
  mobilizer_->SetAngle(context_.get(), new_angle);
  X_FM = mobilizer_->CalcAcrossMobilizerTransform(*context_);
  mobilizer_z->update_X_FM(&new_angle, &fast_X_FM);
  EXPECT_TRUE(fast_X_FM.IsNearlyEqualTo(X_FM, kTol));

  TestApplyR_FM(X_FM, *mobilizer_z);
  TestPrePostMultiplyByX_FM(X_FM, *mobilizer_z);
}

TEST_F(RevoluteMobilizerTest, CalcAcrossMobilizerSpatialVeloctiy) {
  const double angular_rate = 1.5;
  const SpatialVelocity<double> V_FM =
      mobilizer_->CalcAcrossMobilizerSpatialVelocity(*context_,
                                                     Vector1d(angular_rate));

  const SpatialVelocity<double> V_FM_expected(
      mobilizer_->revolute_axis() * angular_rate, Vector3d::Zero());

  // Though checked below, we make it explicit here that this mobilizer should
  // introduce no translations at all.
  EXPECT_EQ(V_FM.translational(), Vector3d::Zero());
  EXPECT_TRUE(V_FM.IsApprox(V_FM_expected, kTolerance));
}

TEST_F(RevoluteMobilizerTest, CalcAcrossMobilizerSpatialAcceleration) {
  const double angular_acceleration = 1.5;
  const SpatialAcceleration<double> A_FM =
      mobilizer_->CalcAcrossMobilizerSpatialAcceleration(
          *context_, Vector1d(angular_acceleration));

  const SpatialAcceleration<double> A_FM_expected(
      mobilizer_->revolute_axis() * angular_acceleration, Vector3d::Zero());

  // Though checked below, we make it explicit here that this mobilizer should
  // introduce no translations at all.
  EXPECT_EQ(A_FM.translational(), Vector3d::Zero());
  EXPECT_TRUE(A_FM.IsApprox(A_FM_expected, kTolerance));
}

TEST_F(RevoluteMobilizerTest, ProjectSpatialForce) {
  const Vector3d torque_Mo_F(1.0, 2.0, 3.0);
  const Vector3d force_Mo_F(1.0, 2.0, 3.0);
  const SpatialForce<double> F_Mo_F(torque_Mo_F, force_Mo_F);
  Vector1d tau;
  mobilizer_->ProjectSpatialForce(*context_, F_Mo_F, tau);

  // Only the torque along axis_F does work.
  const double tau_expected = torque_Mo_F.dot(mobilizer_->revolute_axis());
  EXPECT_NEAR(tau(0), tau_expected, kTolerance);
}

TEST_F(RevoluteMobilizerTest, MapVelocityToQDotAndBack) {
  EXPECT_TRUE(mobilizer_->is_velocity_equal_to_qdot());

  Vector1d v(1.5);
  Vector1d qdot;
  mobilizer_->MapVelocityToQDot(*context_, v, &qdot);
  EXPECT_NEAR(qdot(0), v(0), kTolerance);

  qdot(0) = -std::sqrt(2);
  mobilizer_->MapQDotToVelocity(*context_, qdot, &v);
  EXPECT_NEAR(v(0), qdot(0), kTolerance);

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

TEST_F(RevoluteMobilizerTest, KinematicMapping) {
  // For this joint, Nplus = 1 independently of the state. We therefore set the
  // state to NaN in order to verify this.
  tree()
      .GetMutablePositionsAndVelocities(context_.get())
      .setConstant(std::numeric_limits<double>::quiet_NaN());

  // Compute N.
  MatrixX<double> N(1, 1);
  mobilizer_->CalcNMatrix(*context_, &N);
  EXPECT_EQ(N(0, 0), 1.0);

  // Compute Nplus.
  MatrixX<double> Nplus(1, 1);
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);
  EXPECT_EQ(Nplus(0, 0), 1.0);

  // Ensure Ṅ(q,q̇) = [0].
  MatrixX<double> NDot(1, 1);
  mobilizer_->CalcNDotMatrix(*context_, &NDot);
  EXPECT_EQ(NDot(0, 0), 0.0);

  // Ensure Ṅ⁺(q,q̇) = [0].
  MatrixX<double> NplusDot(1, 1);
  mobilizer_->CalcNplusDotMatrix(*context_, &NplusDot);
  EXPECT_EQ(NplusDot(0, 0), 0.0);
}

TEST_F(RevoluteMobilizerTest, MapUsesN) {
  // Set an arbitrary "non-zero" state.
  mobilizer_->SetAngle(context_.get(), 1.5);

  // Set arbitrary v and MapVelocityToQDot
  Vector1d v(2.5);
  Vector1d qdot;
  mobilizer_->MapVelocityToQDot(*context_, v, &qdot);

  // Compute N.
  MatrixX<double> N(1, 1);
  mobilizer_->CalcNMatrix(*context_, &N);

  // Ensure N(q) is used in `q̇ = N(q)⋅v`
  EXPECT_EQ(qdot, N * v);
}

TEST_F(RevoluteMobilizerTest, MapUsesNplus) {
  // Set an arbitrary "non-zero" state.
  mobilizer_->SetAngle(context_.get(), 1.5);

  // Set arbitrary qdot and MapQDotToVelocity
  Vector1d qdot(2.5);
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
