#include "drake/multibody/tree/universal_mobilizer.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/math/rotation_matrix.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/test/mobilizer_tester.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

using Eigen::AngleAxisd;
using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using math::RigidTransformd;
using math::RotationMatrixd;

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();

// Fixture to setup a simple MBT model containing a universal mobilizer.
class UniversalMobilizerTest : public MobilizerTester {
 public:
  // Creates a simple model consisting of a single body with a universal
  // mobilizer connecting it to the world.
  void SetUp() override {
    mobilizer_ =
        &AddMobilizerAndFinalize(std::make_unique<UniversalMobilizer<double>>(
            tree().world_body().body_frame(), body_->body_frame()));
  }

  MatrixXd CalcHMatrix(const Vector2d angles) {
    MatrixXd H = MatrixXd::Zero(6, 2);
    H(0, 0) = 1;
    H(1, 1) = cos(angles[0]);
    H(2, 1) = sin(angles[0]);
    return H;
  }

 protected:
  const UniversalMobilizer<double>* mobilizer_{nullptr};
};

// Verifies method to mutate and access the context.
TEST_F(UniversalMobilizerTest, StateAccess) {
  const Vector2d some_values1(1.5, 2.5);
  const Vector2d some_values2(std::sqrt(2), 3.);
  // Verify we can set a universal mobilizer position given the model's context.
  mobilizer_->set_angles(context_.get(), some_values1);
  EXPECT_EQ(mobilizer_->get_angles(*context_), some_values1);
  mobilizer_->set_angles(context_.get(), some_values2);
  EXPECT_EQ(mobilizer_->get_angles(*context_), some_values2);

  // Verify we can set a universal mobilizer position rate given the model's
  // context.
  mobilizer_->set_angular_rates(context_.get(), some_values1);
  EXPECT_EQ(mobilizer_->get_angular_rates(*context_), some_values1);
  mobilizer_->set_angular_rates(context_.get(), some_values2);
  EXPECT_EQ(mobilizer_->get_angular_rates(*context_), some_values2);
}

TEST_F(UniversalMobilizerTest, ZeroState) {
  const Vector2d some_values1(1.5, 2.5);
  const Vector2d some_values2(std::sqrt(2), 3.);
  // Set the state to some arbitrary non-zero value.
  mobilizer_->set_angles(context_.get(), some_values1);
  mobilizer_->set_angular_rates(context_.get(), some_values2);
  EXPECT_EQ(mobilizer_->get_angles(*context_), some_values1);
  EXPECT_EQ(mobilizer_->get_angular_rates(*context_), some_values2);

  // Set the "zero state" for this mobilizer, which does happen to be that of
  // zero position and velocity.
  mobilizer_->set_zero_state(*context_, &context_->get_mutable_state());
  EXPECT_EQ(mobilizer_->get_angles(*context_), Vector2d::Zero());
  EXPECT_EQ(mobilizer_->get_angular_rates(*context_), Vector2d::Zero());
}

TEST_F(UniversalMobilizerTest, DefaultPosition) {
  UniversalMobilizer<double>* mutable_mobilizer =
      &mutable_tree().get_mutable_variant(*mobilizer_);

  EXPECT_EQ(mobilizer_->get_angles(*context_), Vector2d::Zero());

  Vector2d new_defualt(.4, .5);
  mutable_mobilizer->set_default_position(new_defualt);
  mobilizer_->set_default_state(*context_, &context_->get_mutable_state());

  EXPECT_EQ(mobilizer_->get_angles(*context_), new_defualt);
}

TEST_F(UniversalMobilizerTest, RandomState) {
  RandomGenerator generator;
  std::uniform_real_distribution<symbolic::Expression> uniform;

  UniversalMobilizer<double>* mutable_mobilizer =
      &mutable_tree().get_mutable_variant(*mobilizer_);

  // Default behavior is to set to zero.
  mutable_mobilizer->set_random_state(*context_, &context_->get_mutable_state(),
                                      &generator);
  EXPECT_EQ(mobilizer_->get_angles(*context_), Vector2d::Zero());
  EXPECT_EQ(mobilizer_->get_angular_rates(*context_), Vector2d::Zero());

  // Set position to be random, but not velocity (yet).
  mutable_mobilizer->set_random_position_distribution(
      Vector2<symbolic::Expression>(uniform(generator) + 2.0,
                                    uniform(generator) - 2.0));
  mutable_mobilizer->set_random_state(*context_, &context_->get_mutable_state(),
                                      &generator);
  EXPECT_GE(mobilizer_->get_angles(*context_)[0], 2.0);
  EXPECT_LE(mobilizer_->get_angles(*context_)[0], 3.0);
  EXPECT_GE(mobilizer_->get_angles(*context_)[1], -2.0);
  EXPECT_LE(mobilizer_->get_angles(*context_)[1], -1.0);
  EXPECT_EQ(mobilizer_->get_angular_rates(*context_), Vector2d::Zero());

  // Set the velocity distribution.  Now both should be random.
  mutable_mobilizer->set_random_velocity_distribution(
      Vector2<symbolic::Expression>(uniform(generator) - 3.0,
                                    uniform(generator) + 3.0));
  mutable_mobilizer->set_random_state(*context_, &context_->get_mutable_state(),
                                      &generator);
  EXPECT_GE(mobilizer_->get_angles(*context_)[0], 2.0);
  EXPECT_LE(mobilizer_->get_angles(*context_)[0], 3.0);
  EXPECT_GE(mobilizer_->get_angles(*context_)[1], -2.0);
  EXPECT_LE(mobilizer_->get_angles(*context_)[1], -1.0);
  EXPECT_GE(mobilizer_->get_angular_rates(*context_)[0], -3.0);
  EXPECT_LE(mobilizer_->get_angular_rates(*context_)[0], -2.0);
  EXPECT_GE(mobilizer_->get_angular_rates(*context_)[1], 3.0);
  EXPECT_LE(mobilizer_->get_angular_rates(*context_)[1], 4.0);

  // Check that they change on a second draw from the distribution.
  const Vector2d last_angles = mobilizer_->get_angles(*context_);
  const Vector2d last_angular_rates = mobilizer_->get_angular_rates(*context_);
  mutable_mobilizer->set_random_state(*context_, &context_->get_mutable_state(),
                                      &generator);
  EXPECT_NE(mobilizer_->get_angles(*context_), last_angles);
  EXPECT_NE(mobilizer_->get_angular_rates(*context_), last_angular_rates);
}

TEST_F(UniversalMobilizerTest, CalcAcrossMobilizerTransform) {
  const Vector2d angles(1, 0.5);
  mobilizer_->set_angles(context_.get(), angles);
  const RigidTransformd X_FM(
      mobilizer_->CalcAcrossMobilizerTransform(*context_));

  const RotationMatrixd R_FI = RotationMatrixd::MakeXRotation(angles[0]);
  const RotationMatrixd R_IM = RotationMatrixd::MakeYRotation(angles[1]);
  const RigidTransformd X_FM_expected(R_FI * R_IM);

  // Though checked below, we make it explicit here that this mobilizer should
  // introduce no translations at all.
  EXPECT_EQ(X_FM.translation(), Vector3d::Zero());
  EXPECT_TRUE(CompareMatrices(X_FM.GetAsMatrix34(),
                              X_FM_expected.GetAsMatrix34(), kTolerance,
                              MatrixCompareType::relative));
}

TEST_F(UniversalMobilizerTest, CalcAcrossMobilizerSpatialVeloctiy) {
  const Vector2d angles(1, 0.5);
  const Vector2d angular_rates(1.5, 2);
  mobilizer_->set_angles(context_.get(), angles);
  const SpatialVelocity<double> V_FM =
      mobilizer_->CalcAcrossMobilizerSpatialVelocity(*context_, angular_rates);

  const MatrixXd H_expected = CalcHMatrix(angles);
  const SpatialVelocity<double> V_FM_expected(H_expected * angular_rates);

  // Though checked below, we make it explicit here that this mobilizer should
  // introduce no translations at all.
  EXPECT_EQ(V_FM.translational(), Vector3d::Zero());
  EXPECT_TRUE(V_FM.IsApprox(V_FM_expected, kTolerance));
}

TEST_F(UniversalMobilizerTest, CalcAcrossMobilizerSpatialAcceleration) {
  const Vector2d angles(1, 0.5);
  const Vector2d angular_rates(1.5, 2);
  const Vector2d angular_acceleration(3, 2.5);
  mobilizer_->set_angles(context_.get(), angles);
  mobilizer_->set_angular_rates(context_.get(), angular_rates);

  const SpatialAcceleration<double> A_FM =
      mobilizer_->CalcAcrossMobilizerSpatialAcceleration(*context_,
                                                         angular_acceleration);

  const MatrixXd H_expected = CalcHMatrix(angles);
  MatrixXd H_dot_expected = MatrixXd::Zero(6, 2);
  H_dot_expected(1, 1) = -angular_rates[0] * sin(angles[0]);
  H_dot_expected(2, 1) = angular_rates[0] * cos(angles[0]);
  const SpatialAcceleration<double> A_FM_expected(
      H_expected * angular_acceleration + H_dot_expected * angular_rates);

  // Though checked below, we make it explicit here that this mobilizer should
  // introduce no translations at all.
  EXPECT_EQ(A_FM.translational(), Vector3d::Zero());
  EXPECT_TRUE(A_FM.IsApprox(A_FM_expected, kTolerance));
}

TEST_F(UniversalMobilizerTest, ProjectSpatialForce) {
  const Vector2d angles(1, 0.5);
  mobilizer_->set_angles(context_.get(), angles);

  const Vector3d torque_Mo_F(1.0, 2.0, 3.0);
  const Vector3d force_Mo_F(1.0, 2.0, 3.0);
  const SpatialForce<double> F_Mo_F(torque_Mo_F, force_Mo_F);
  Vector2d tau;
  mobilizer_->ProjectSpatialForce(*context_, F_Mo_F, tau);

  const MatrixXd H_expected = CalcHMatrix(angles);
  const Vector2d tau_expected = H_expected.topRows(3).transpose() * torque_Mo_F;
  EXPECT_TRUE(CompareMatrices(tau, tau_expected, kTolerance,
                              MatrixCompareType::relative));
}

TEST_F(UniversalMobilizerTest, MapVelocityToQDotAndBack) {
  Vector2d v(1.5, 2.5);
  Vector2d qdot;
  mobilizer_->MapVelocityToQDot(*context_, v, &qdot);
  EXPECT_TRUE(
      CompareMatrices(qdot, v, kTolerance, MatrixCompareType::relative));

  qdot(0) = -std::sqrt(2);
  mobilizer_->MapQDotToVelocity(*context_, qdot, &v);
  EXPECT_TRUE(
      CompareMatrices(v, qdot, kTolerance, MatrixCompareType::relative));
}

TEST_F(UniversalMobilizerTest, KinematicMapping) {
  // For this joint, Nplus = I independently of the state. We therefore set the
  // state to NaN in order to verify this.
  tree()
      .GetMutablePositionsAndVelocities(context_.get())
      .setConstant(std::numeric_limits<double>::quiet_NaN());

  // Compute N.
  MatrixX<double> N(2, 2);
  mobilizer_->CalcNMatrix(*context_, &N);
  EXPECT_EQ(N, Matrix2d::Identity());

  // Compute Nplus.
  MatrixX<double> Nplus(2, 2);
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);
  EXPECT_EQ(Nplus, Matrix2d::Identity());
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
