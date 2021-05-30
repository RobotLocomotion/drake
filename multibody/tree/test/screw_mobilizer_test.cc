#include "drake/multibody/tree/screw_mobilizer.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/multibody_tree-inl.h"
#include "drake/multibody/tree/multibody_tree_system.h"
#include "drake/multibody/tree/test/mobilizer_tester.h"
#include "drake/systems/framework/context.h"

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

// Fixture to setup a simple MBT model containing a screw mobilizer.
class ScrewMobilizerTest : public MobilizerTester {
 public:
  // Creates a simple model consisting of a single body with a screw
  // mobilizer connecting it to the world.
  void SetUp() override {
    mobilizer_ =
        &AddMobilizerAndFinalize(std::make_unique<ScrewMobilizer<double>>(
            tree().world_body().body_frame(), body_->body_frame(), kScrewPitch));
  }

 protected:
  const ScrewMobilizer<double>* mobilizer_{nullptr};
};

// Verifies method to mutate and access the context.
TEST_F(ScrewMobilizerTest, StateAccess) {
  const double translation_z_first{1.};
  const double translation_z_second{2.};
  const double angle_z_first{1. * 180. / M_PI};
  const double angle_z_second{2. * 180. / M_PI};

  // Verify we can set a screw mobilizer configuration given the model's
  // context.
  mobilizer_->set_translation(context_.get(), translation_z_first);
  EXPECT_EQ(mobilizer_->get_translation(*context_), translation_z_first);

  mobilizer_->set_translation(context_.get(), translation_z_second);
  EXPECT_EQ(mobilizer_->get_translation(*context_), translation_z_second);

  mobilizer_->set_angle(context_.get(), angle_z_first);
  EXPECT_EQ(mobilizer_->get_angle(*context_), angle_z_first);

  mobilizer_->set_angle(context_.get(), angle_z_second);
  EXPECT_EQ(mobilizer_->get_angle(*context_), angle_z_second);

  // Verify we can set a screw mobilizer velocities given the model's context.
  mobilizer_->set_translation_rate(context_.get(), translation_z_first);
  EXPECT_EQ(mobilizer_->get_translation_rate(*context_), translation_z_first);

  mobilizer_->set_translation_rate(context_.get(), translation_z_second);
  EXPECT_EQ(mobilizer_->get_translation_rate(*context_), translation_z_second);

  mobilizer_->set_angular_rate(context_.get(), angle_z_first);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), angle_z_first);

  mobilizer_->set_angular_rate(context_.get(), angle_z_second);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), angle_z_second);
}

TEST_F(ScrewMobilizerTest, ZeroState) {
  const double angle_z_first{1. * 180. / M_PI};
  const double angle_z_second{2. * 180. / M_PI};

  const double derived_translation_z_first{angle_z_first /
                                           (2 * M_PI) * kScrewPitch};
  const double derived_translation_z_second{angle_z_second /
                                            (2 * M_PI) * kScrewPitch};

  // Set the state to some arbitrary non-zero value.
  mobilizer_->set_angle(context_.get(), angle_z_first);
  mobilizer_->set_angular_rate(context_.get(), angle_z_second);
  EXPECT_EQ(mobilizer_->get_angle(*context_), angle_z_first);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), angle_z_second);
  EXPECT_EQ(mobilizer_->get_translation(*context_), derived_translation_z_first);
  EXPECT_EQ(mobilizer_->get_translation_rate(*context_), derived_translation_z_second);

  // Set the "zero state" for this mobilizer, which does happen to be that of
  // zero position and velocity.
  mobilizer_->set_zero_state(*context_, &context_->get_mutable_state());
  EXPECT_EQ(mobilizer_->get_translation(*context_), 0.0);
  EXPECT_EQ(mobilizer_->get_translation_rate(*context_), 0.0);
  EXPECT_EQ(mobilizer_->get_angle(*context_), 0.0);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), 0.0);
}

TEST_F(ScrewMobilizerTest, DefaultPosition) {
  ScrewMobilizer<double>* mutable_mobilizer =
      &mutable_tree().get_mutable_variant(*mobilizer_);

  EXPECT_EQ(mobilizer_->get_translation(*context_), 0.0);
  EXPECT_EQ(mobilizer_->get_angle(*context_), 0.0);

  const Vector1d new_default(.4);
  mutable_mobilizer->set_default_position(new_default);
  mobilizer_->set_default_state(*context_, &context_->get_mutable_state());

  EXPECT_EQ(mobilizer_->get_angle(*context_), new_default(0));
  EXPECT_EQ(mobilizer_->get_translation(*context_),
            new_default(0) * kScrewPitch / (2. * M_PI));
}

TEST_F(ScrewMobilizerTest, RandomState) {
  RandomGenerator generator;
  std::uniform_real_distribution<symbolic::Expression> uniform;

  ScrewMobilizer<double>* mutable_mobilizer =
      &mutable_tree().get_mutable_variant(*mobilizer_);

  // Default behavior is to set to zero.
  mutable_mobilizer->set_random_state(*context_, &context_->get_mutable_state(),
                                      &generator);
  EXPECT_EQ(mobilizer_->get_angle(*context_), 0.0);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), 0.0);

  // Set position to be random, but not velocity (yet).
  mutable_mobilizer->set_random_position_distribution(
      Vector1<symbolic::Expression>(uniform(generator) + 1.0));
  mutable_mobilizer->set_random_state(*context_, &context_->get_mutable_state(),
      &generator);
  EXPECT_GE(mobilizer_->get_angle(*context_), 1.0);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), 0.0);

  // Set the velocity distribution.  Now both should be random.
  mutable_mobilizer->set_random_velocity_distribution(
      Vector1<symbolic::Expression>(uniform(generator) - 1.0));
  mutable_mobilizer->set_random_state(*context_, &context_->get_mutable_state(),
      &generator);
  EXPECT_GE(mobilizer_->get_angle(*context_), 1.0);
  EXPECT_GE(mobilizer_->get_angular_rate(*context_), -1.0);

  // Check that they change on a second draw from the distribution.
  const double last_translation = mobilizer_->get_translation(*context_);
  const double last_translational_rate = mobilizer_->get_translation_rate(
      *context_);
  const double last_angle = mobilizer_->get_angle(*context_);
  const double last_angular_rate = mobilizer_->get_angular_rate(*context_);
  mutable_mobilizer->set_random_state(*context_, &context_->get_mutable_state(),
                                      &generator);
  EXPECT_NE(mobilizer_->get_translation(*context_), last_translation);
  EXPECT_NE(mobilizer_->get_translation_rate(*context_),
    last_translational_rate);
  EXPECT_NE(mobilizer_->get_angle(*context_), last_angle);
  EXPECT_NE(mobilizer_->get_angular_rate(*context_), last_angular_rate);
}

TEST_F(ScrewMobilizerTest, CalcAcrossMobilizerTransform) {
  const double angle = 1.5;
  mobilizer_->set_angle(context_.get(), angle);
  const RigidTransformd X_FM(
      mobilizer_->CalcAcrossMobilizerTransform(*context_));

  Vector3d X_FM_translation;
  X_FM_translation << 0.0, 0.0, angle / (2 * M_PI) * kScrewPitch;
  const RigidTransformd X_FM_expected(RotationMatrixd::MakeZRotation(angle),
                                      X_FM_translation);

  EXPECT_TRUE(CompareMatrices(X_FM.GetAsMatrix34(),
                              X_FM_expected.GetAsMatrix34(), kTolerance,
                              MatrixCompareType::relative));
}

TEST_F(ScrewMobilizerTest, CalcAcrossMobilizerSpatialVeloctiy) {
  const double angle = 1.5;
  const double angular_velocity = 0.1;
  const Vector1d spatial_velocity(angular_velocity / (2 * M_PI) * kScrewPitch);
  mobilizer_->set_angle(context_.get(), angle);
  const SpatialVelocity<double> V_FM =
      mobilizer_->CalcAcrossMobilizerSpatialVelocity(*context_,
                                                     spatial_velocity);

  VectorXd v_expected(6);
  v_expected << 0.0, 0.0, angular_velocity, 0.0, 0.0, spatial_velocity(0);
  const SpatialVelocity<double> V_FM_expected(v_expected);

  EXPECT_TRUE(V_FM.IsApprox(V_FM_expected, kTolerance));
}

TEST_F(ScrewMobilizerTest, CalcAcrossMobilizerSpatialAcceleration) {
  const double angle = 1.5;
  const double angle_rate = 3;
  const Vector1d angle_acceleration(4.5);
  mobilizer_->set_angle(context_.get(), angle);
  mobilizer_->set_angular_rate(context_.get(), angle_rate);

  const SpatialAcceleration<double> A_FM =
      mobilizer_->CalcAcrossMobilizerSpatialAcceleration(*context_,
                                                         angle_acceleration);

  VectorXd a_expected(6);
  a_expected << 0.0, 0.0, angle_acceleration[0], 0.0, 0.0,
                angle_acceleration[0] / (2 * M_PI) * kScrewPitch;
  const SpatialAcceleration<double> A_FM_expected(a_expected);

  EXPECT_TRUE(A_FM.IsApprox(A_FM_expected, kTolerance));
}

TEST_F(ScrewMobilizerTest, ProjectSpatialForce) {
  const double translation(0.5);
  const double angle = 1.5;
  mobilizer_->set_translation(context_.get(), translation);
  mobilizer_->set_angle(context_.get(), angle);

  const Vector3d torque_Mo_F(1.0, 2.0, 3.0);
  const Vector3d force_Mo_F(1.0, 2.0, 3.0);
  const SpatialForce<double> F_Mo_F(torque_Mo_F, force_Mo_F);
  Vector1d tau;
  mobilizer_->ProjectSpatialForce(*context_, F_Mo_F, tau);

  const Vector1d tau_expected(torque_Mo_F[2]);
  EXPECT_TRUE(CompareMatrices(tau, tau_expected, kTolerance,
                              MatrixCompareType::relative));
}

TEST_F(ScrewMobilizerTest, MapVelocityToQDotAndBack) {
  Vector1d v(1.5);
  Vector1d qdot;
  mobilizer_->MapVelocityToQDot(*context_, v, &qdot);
  EXPECT_TRUE(
      CompareMatrices(qdot, v, kTolerance, MatrixCompareType::relative));

  qdot(0) = -std::sqrt(2);
  mobilizer_->MapQDotToVelocity(*context_, qdot, &v);
  EXPECT_TRUE(
      CompareMatrices(v, qdot, kTolerance, MatrixCompareType::relative));
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
  EXPECT_EQ(N, Matrix1d::Identity());

  // Compute Nplus.
  MatrixX<double> Nplus(1, 1);
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);
  EXPECT_EQ(Nplus, Matrix1d::Identity());
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
