#include "drake/multibody/tree/planar_mobilizer.h"

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

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using math::RigidTransformd;
using math::RotationMatrixd;

constexpr double kTolerance = 10 * std::numeric_limits<double>::epsilon();

// Fixture to setup a simple MBT model containing a planar mobilizer.
class PlanarMobilizerTest : public MobilizerTester {
 public:
  // Creates a simple model consisting of a single body with a planar
  // mobilizer connecting it to the world.
  void SetUp() override {
    mobilizer_ =
        &AddMobilizerAndFinalize(std::make_unique<PlanarMobilizer<double>>(
            tree().world_body().body_frame(), body_->body_frame()));
  }

 protected:
  const PlanarMobilizer<double>* mobilizer_{nullptr};
};

// Verifies method to mutate and access the context.
TEST_F(PlanarMobilizerTest, StateAccess) {
  const Vector2d some_values1(1.5, 2.5);
  const Vector2d some_values2(std::sqrt(2), 3.);
  const double some_values3 = 4.5;
  const double some_values4 = std::sqrt(5);
  // Verify we can set a planar mobilizer configuration given the model's
  // context.
  mobilizer_->set_translations(context_.get(), some_values1);
  EXPECT_EQ(mobilizer_->get_translations(*context_), some_values1);
  mobilizer_->set_translations(context_.get(), some_values2);
  EXPECT_EQ(mobilizer_->get_translations(*context_), some_values2);
  mobilizer_->set_angle(context_.get(), some_values3);
  EXPECT_EQ(mobilizer_->get_angle(*context_), some_values3);
  mobilizer_->set_angle(context_.get(), some_values4);
  EXPECT_EQ(mobilizer_->get_angle(*context_), some_values4);

  // Verify we can set a planar mobilizer velocities given the model's context.
  mobilizer_->set_translation_rates(context_.get(), some_values1);
  EXPECT_EQ(mobilizer_->get_translation_rates(*context_), some_values1);
  mobilizer_->set_translation_rates(context_.get(), some_values2);
  EXPECT_EQ(mobilizer_->get_translation_rates(*context_), some_values2);
  mobilizer_->set_angular_rate(context_.get(), some_values3);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), some_values3);
  mobilizer_->set_angular_rate(context_.get(), some_values4);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), some_values4);
}

TEST_F(PlanarMobilizerTest, ZeroState) {
  const Vector2d some_values1(1.5, 2.5);
  const Vector2d some_values2(std::sqrt(2), 3.);
  const double some_values3 = 4.5;
  const double some_values4 = std::sqrt(5);
  // Set the state to some arbitrary non-zero value.
  mobilizer_->set_translations(context_.get(), some_values1);
  mobilizer_->set_translation_rates(context_.get(), some_values2);
  mobilizer_->set_angle(context_.get(), some_values3);
  mobilizer_->set_angular_rate(context_.get(), some_values4);
  EXPECT_EQ(mobilizer_->get_translations(*context_), some_values1);
  EXPECT_EQ(mobilizer_->get_translation_rates(*context_), some_values2);
  EXPECT_EQ(mobilizer_->get_angle(*context_), some_values3);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), some_values4);

  // Set the "zero state" for this mobilizer, which does happen to be that of
  // zero position and velocity.
  mobilizer_->set_zero_state(*context_, &context_->get_mutable_state());
  EXPECT_EQ(mobilizer_->get_translations(*context_), Vector2d::Zero());
  EXPECT_EQ(mobilizer_->get_translation_rates(*context_), Vector2d::Zero());
  EXPECT_EQ(mobilizer_->get_angle(*context_), 0.0);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), 0.0);
}

TEST_F(PlanarMobilizerTest, DefaultPosition) {
  PlanarMobilizer<double>* mutable_mobilizer =
      &mutable_tree().get_mutable_variant(*mobilizer_);

  EXPECT_EQ(mobilizer_->get_translations(*context_), Vector2d::Zero());
  EXPECT_EQ(mobilizer_->get_angle(*context_), 0.0);

  Vector3d new_default(.4, .5, .6);
  mutable_mobilizer->set_default_position(new_default);
  mobilizer_->set_default_state(*context_, &context_->get_mutable_state());

  EXPECT_EQ(mobilizer_->get_translations(*context_), new_default.head(2));
  EXPECT_EQ(mobilizer_->get_angle(*context_), new_default[2]);
}

TEST_F(PlanarMobilizerTest, RandomState) {
  RandomGenerator generator;
  std::uniform_real_distribution<symbolic::Expression> uniform;

  PlanarMobilizer<double>* mutable_mobilizer =
      &mutable_tree().get_mutable_variant(*mobilizer_);

  // Default behavior is to set to zero.
  mutable_mobilizer->set_random_state(*context_, &context_->get_mutable_state(),
                                      &generator);
  EXPECT_EQ(mobilizer_->get_translations(*context_), Vector2d::Zero());
  EXPECT_EQ(mobilizer_->get_translation_rates(*context_), Vector2d::Zero());
  EXPECT_EQ(mobilizer_->get_angle(*context_), 0.0);
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), 0.0);

  // Set position to be random, but not velocity (yet).
  mutable_mobilizer->set_random_position_distribution(
      Vector3<symbolic::Expression>(uniform(generator) + 1.0,
                                    uniform(generator) + 2.0,
                                    uniform(generator) + 3.0));
  mutable_mobilizer->set_random_state(*context_, &context_->get_mutable_state(),
                                      &generator);
  EXPECT_GE(mobilizer_->get_translations(*context_)[0], 1.0);
  EXPECT_LE(mobilizer_->get_translations(*context_)[0], 2.0);
  EXPECT_GE(mobilizer_->get_translations(*context_)[1], 2.0);
  EXPECT_LE(mobilizer_->get_translations(*context_)[1], 3.0);
  EXPECT_GE(mobilizer_->get_angle(*context_), 3.0);
  EXPECT_LE(mobilizer_->get_angle(*context_), 4.0);
  EXPECT_EQ(mobilizer_->get_translation_rates(*context_), Vector2d::Zero());
  EXPECT_EQ(mobilizer_->get_angular_rate(*context_), 0.0);

  // Set the velocity distribution.  Now both should be random.
  mutable_mobilizer->set_random_velocity_distribution(
      Vector3<symbolic::Expression>(uniform(generator) - 2.0,
                                    uniform(generator) - 3.0,
                                    uniform(generator) - 4.0));
  mutable_mobilizer->set_random_state(*context_, &context_->get_mutable_state(),
                                      &generator);
  EXPECT_GE(mobilizer_->get_translations(*context_)[0], 1.0);
  EXPECT_LE(mobilizer_->get_translations(*context_)[0], 2.0);
  EXPECT_GE(mobilizer_->get_translations(*context_)[1], 2.0);
  EXPECT_LE(mobilizer_->get_translations(*context_)[1], 3.0);
  EXPECT_GE(mobilizer_->get_angle(*context_), 3.0);
  EXPECT_LE(mobilizer_->get_angle(*context_), 4.0);
  EXPECT_GE(mobilizer_->get_translation_rates(*context_)[0], -2.0);
  EXPECT_LE(mobilizer_->get_translation_rates(*context_)[0], -1.0);
  EXPECT_GE(mobilizer_->get_translation_rates(*context_)[1], -3.0);
  EXPECT_LE(mobilizer_->get_translation_rates(*context_)[1], -2.0);
  EXPECT_GE(mobilizer_->get_angular_rate(*context_), -4.0);
  EXPECT_LE(mobilizer_->get_angular_rate(*context_), -3.0);

  // Check that they change on a second draw from the distribution.
  const Vector2d last_translations = mobilizer_->get_translations(*context_);
  const Vector2d last_translational_rates =
      mobilizer_->get_translation_rates(*context_);
  const double last_angle = mobilizer_->get_angle(*context_);
  const double last_angular_rates = mobilizer_->get_angular_rate(*context_);
  mutable_mobilizer->set_random_state(*context_, &context_->get_mutable_state(),
                                      &generator);
  EXPECT_NE(mobilizer_->get_translations(*context_), last_translations);
  EXPECT_NE(mobilizer_->get_translation_rates(*context_),
            last_translational_rates);
  EXPECT_NE(mobilizer_->get_angle(*context_), last_angle);
  EXPECT_NE(mobilizer_->get_angular_rate(*context_), last_angular_rates);
}

TEST_F(PlanarMobilizerTest, CalcAcrossMobilizerTransform) {
  const Vector2d translations(1, 0.5);
  const double angle = 1.5;
  mobilizer_->set_translations(context_.get(), translations);
  mobilizer_->set_angle(context_.get(), angle);
  const RigidTransformd X_FM(
      mobilizer_->CalcAcrossMobilizerTransform(*context_));

  Vector3d X_FM_translation;
  X_FM_translation << translations[0], translations[1], 0.0;
  const RigidTransformd X_FM_expected(RotationMatrixd::MakeZRotation(angle),
                                      X_FM_translation);

  EXPECT_TRUE(CompareMatrices(X_FM.GetAsMatrix34(),
                              X_FM_expected.GetAsMatrix34(), kTolerance,
                              MatrixCompareType::relative));
}

TEST_F(PlanarMobilizerTest, CalcAcrossMobilizerSpatialVeloctiy) {
  const Vector2d translations(1, 0.5);
  const double angle = 1.5;
  const Vector3d velocity(2.5, 2, 3);
  mobilizer_->set_translations(context_.get(), translations);
  mobilizer_->set_angle(context_.get(), angle);
  const SpatialVelocity<double> V_FM =
      mobilizer_->CalcAcrossMobilizerSpatialVelocity(*context_, velocity);

  VectorXd v_expected(6);
  v_expected << 0.0, 0.0, velocity[2], velocity[0], velocity[1], 0.0;
  const SpatialVelocity<double> V_FM_expected(v_expected);

  EXPECT_TRUE(V_FM.IsApprox(V_FM_expected, kTolerance));
}

TEST_F(PlanarMobilizerTest, CalcAcrossMobilizerSpatialAcceleration) {
  const Vector2d translations(1, 0.5);
  const double angle = 1.5;
  const Vector2d translation_rates(2.5, 2);
  const double angle_rate = 3;
  const Vector3d acceleration(3.5, 4, 4.5);
  mobilizer_->set_translations(context_.get(), translations);
  mobilizer_->set_angle(context_.get(), angle);
  mobilizer_->set_translation_rates(context_.get(), translation_rates);
  mobilizer_->set_angular_rate(context_.get(), angle_rate);

  const SpatialAcceleration<double> A_FM =
      mobilizer_->CalcAcrossMobilizerSpatialAcceleration(*context_,
                                                         acceleration);

  VectorXd a_expected(6);
  a_expected << 0.0, 0.0, acceleration[2], acceleration[0], acceleration[1],
      0.0;
  const SpatialAcceleration<double> A_FM_expected(a_expected);

  EXPECT_TRUE(A_FM.IsApprox(A_FM_expected, kTolerance));
}

TEST_F(PlanarMobilizerTest, ProjectSpatialForce) {
  const Vector2d translations(1, 0.5);
  const double angle = 1.5;
  mobilizer_->set_translations(context_.get(), translations);
  mobilizer_->set_angle(context_.get(), angle);

  const Vector3d torque_Mo_F(1.0, 2.0, 3.0);
  const Vector3d force_Mo_F(1.0, 2.0, 3.0);
  const SpatialForce<double> F_Mo_F(torque_Mo_F, force_Mo_F);
  Vector3d tau;
  mobilizer_->ProjectSpatialForce(*context_, F_Mo_F, tau);

  const Vector3d tau_expected(force_Mo_F[0], force_Mo_F[1], torque_Mo_F[2]);
  EXPECT_TRUE(CompareMatrices(tau, tau_expected, kTolerance,
                              MatrixCompareType::relative));
}

TEST_F(PlanarMobilizerTest, MapVelocityToQDotAndBack) {
  Vector3d v(1.5, 2.5, 3.5);
  Vector3d qdot;
  mobilizer_->MapVelocityToQDot(*context_, v, &qdot);
  EXPECT_TRUE(
      CompareMatrices(qdot, v, kTolerance, MatrixCompareType::relative));

  qdot(0) = -std::sqrt(2);
  mobilizer_->MapQDotToVelocity(*context_, qdot, &v);
  EXPECT_TRUE(
      CompareMatrices(v, qdot, kTolerance, MatrixCompareType::relative));
}

TEST_F(PlanarMobilizerTest, KinematicMapping) {
  // For this joint, Nplus = I independently of the state. We therefore set the
  // state to NaN in order to verify this.
  tree()
      .GetMutablePositionsAndVelocities(context_.get())
      .setConstant(std::numeric_limits<double>::quiet_NaN());

  // Compute N.
  MatrixX<double> N(3, 3);
  mobilizer_->CalcNMatrix(*context_, &N);
  EXPECT_EQ(N, Matrix3d::Identity());

  // Compute Nplus.
  MatrixX<double> Nplus(3, 3);
  mobilizer_->CalcNplusMatrix(*context_, &Nplus);
  EXPECT_EQ(Nplus, Matrix3d::Identity());
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
