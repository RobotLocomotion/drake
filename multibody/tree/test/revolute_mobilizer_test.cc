#include "drake/multibody/tree/revolute_mobilizer.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/test/mobilizer_tester.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
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
    mobilizer_ = &AddMobilizerAndFinalize(
        std::make_unique<RevoluteMobilizer<double>>(
            tree().world_body().body_frame(), body_->body_frame(), axis_F_));
  }

 protected:
  const RevoluteMobilizer<double>* mobilizer_{nullptr};
  const Vector3d axis_F_{1.0, 2.0, 3.0};
};

// Verify that RevoluteMobilizer normalizes its axis on construction.
TEST_F(RevoluteMobilizerTest, AxisIsNormalizedAtConstruction) {
  EXPECT_TRUE(CompareMatrices(
      mobilizer_->revolute_axis(), axis_F_.normalized(),
      kTolerance, MatrixCompareType::relative));
}

// Verifies method to mutate and access the context.
TEST_F(RevoluteMobilizerTest, StateAccess) {
  const double some_value1 = 1.5;
  const double some_value2 = std::sqrt(2);
  // Verify we can set a revolute mobilizer position given the model's context.
  mobilizer_->set_angle(context_.get(), some_value1);
  EXPECT_EQ(mobilizer_->get_angle(*context_), some_value1);
  mobilizer_->set_angle(context_.get(), some_value2);
  EXPECT_EQ(mobilizer_->get_angle(*context_), some_value2);

  // Verify we can set a revolute mobilizer position rate given the model's
  // context.
  mobilizer_->set_angular_rate(context_.get(), some_value1);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), some_value1);
  mobilizer_->set_angular_rate(context_.get(), some_value2);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), some_value2);
}

TEST_F(RevoluteMobilizerTest, ZeroState) {
  const double some_value1 = 1.5;
  const double some_value2 = std::sqrt(2);
  // Set the state to some arbitrary non-zero value.
  mobilizer_->set_angle(context_.get(), some_value1);
  EXPECT_EQ(mobilizer_->get_angle(*context_), some_value1);
  mobilizer_->set_angular_rate(context_.get(), some_value2);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), some_value2);

  // Set the "zero state" for this mobilizer, which does happen to be that of
  // zero position and velocity.
  mobilizer_->set_zero_state(*context_, &context_->get_mutable_state());
  EXPECT_EQ(mobilizer_->get_angle(*context_), 0);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), 0);
}

TEST_F(RevoluteMobilizerTest, CalcAcrossMobilizerTransform) {
  const double angle = 1.5;
  mobilizer_->set_angle(context_.get(), angle);
  const RigidTransformd X_FM(
      mobilizer_->CalcAcrossMobilizerTransform(*mbt_context_));

  const RigidTransformd X_FM_expected(
      RotationMatrixd(AngleAxisd(angle, axis_F_.normalized())));

  // Though checked below, we make it explicit here that this mobilizer should
  // introduce no translations at all.
  EXPECT_EQ(X_FM.translation(), Vector3d::Zero());
  EXPECT_TRUE(CompareMatrices(X_FM.GetAsMatrix34(),
                              X_FM_expected.GetAsMatrix34(),
                              kTolerance, MatrixCompareType::relative));
}

TEST_F(RevoluteMobilizerTest, CalcAcrossMobilizerSpatialVeloctiy) {
  const double angular_rate = 1.5;
  const SpatialVelocity<double> V_FM =
      mobilizer_->CalcAcrossMobilizerSpatialVelocity(
          *mbt_context_, Vector1d(angular_rate));

  const SpatialVelocity<double> V_FM_expected(
      axis_F_.normalized() * angular_rate, Vector3d::Zero());

  // Though checked below, we make it explicit here that this mobilizer should
  // introduce no translations at all.
  EXPECT_EQ(V_FM.translational(), Vector3d::Zero());
  EXPECT_TRUE(V_FM.IsApprox(V_FM_expected, kTolerance));
}

TEST_F(RevoluteMobilizerTest, CalcAcrossMobilizerSpatialAcceleration) {
  const double angular_acceleration = 1.5;
  const SpatialAcceleration<double> A_FM =
      mobilizer_->CalcAcrossMobilizerSpatialAcceleration(
          *mbt_context_, Vector1d(angular_acceleration));

  const SpatialAcceleration<double> A_FM_expected(
      axis_F_.normalized() * angular_acceleration, Vector3d::Zero());

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
  mobilizer_->ProjectSpatialForce(*mbt_context_, F_Mo_F, tau);

  // Only the torque along axis_F does work.
  const double tau_expected = torque_Mo_F.dot(axis_F_.normalized());
  EXPECT_NEAR(tau(0), tau_expected, kTolerance);
}

TEST_F(RevoluteMobilizerTest, MapVelocityToQDotAndBack) {
  Vector1d v(1.5);
  Vector1d qdot;
  mobilizer_->MapVelocityToQDot(*mbt_context_, v, &qdot);
  EXPECT_NEAR(qdot(0), v(0), kTolerance);

  qdot(0) = -std::sqrt(2);
  mobilizer_->MapQDotToVelocity(*mbt_context_, qdot, &v);
  EXPECT_NEAR(v(0), qdot(0), kTolerance);
}

TEST_F(RevoluteMobilizerTest, KinematicMapping) {
  // For this joint, Nplus = 1 independently of the state. We therefore set the
  // state to NaN in order to verify this.
  tree().GetMutablePositionsAndVelocities(context_.get()).
      setConstant(std::numeric_limits<double>::quiet_NaN());

  // Compute N.
  MatrixX<double> N(1, 1);
  mobilizer_->CalcNMatrix(*mbt_context_, &N);
  EXPECT_EQ(N(0, 0), 1.0);

  // Compute Nplus.
  MatrixX<double> Nplus(1, 1);
  mobilizer_->CalcNplusMatrix(*mbt_context_, &Nplus);
  EXPECT_EQ(Nplus(0, 0), 1.0);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
