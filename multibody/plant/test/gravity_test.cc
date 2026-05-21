#include <cmath>
#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/spatial_inertia.h"

using Eigen::Vector3d;
using Eigen::VectorXd;

namespace drake {

using multibody::Parser;
using systems::Context;
using systems::Diagram;

namespace multibody {
namespace {

class MultibodyPlantGravityForceTest : public ::testing::Test {
 public:
  void SetUp() override {
    // Load two Iiwa models.
    const std::string url =
        "package://drake_models/iiwa_description/sdf/iiwa14_no_collision.sdf";
    plant_ = std::make_unique<MultibodyPlant<double>>(0.0);

    // Add the model twice.
    iiwa1_ = Parser(plant_.get(), "iiwa1").AddModelsFromUrl(url).at(0);
    iiwa2_ = Parser(plant_.get(), "iiwa2").AddModelsFromUrl(url).at(0);
    plant_->WeldFrames(plant_->world_frame(),
                       plant_->GetFrameByName("iiwa_link_0", iiwa1_));
    plant_->WeldFrames(plant_->world_frame(),
                       plant_->GetFrameByName("iiwa_link_0", iiwa2_));
  }

 protected:
  std::unique_ptr<MultibodyPlant<double>> plant_{nullptr};
  ModelInstanceIndex iiwa1_;
  ModelInstanceIndex iiwa2_;
};

TEST_F(MultibodyPlantGravityForceTest,
       UniformGravityFieldElementApisToDisableGravity) {
  UniformGravityFieldElement<double>& gravity = plant_->mutable_gravity_field();
  // Enabled by default.
  EXPECT_TRUE(gravity.is_enabled(iiwa1_));
  EXPECT_TRUE(gravity.is_enabled(iiwa2_));

  // Disable gravity.
  gravity.set_enabled(iiwa1_, false);
  EXPECT_FALSE(gravity.is_enabled(iiwa1_));
  EXPECT_TRUE(gravity.is_enabled(iiwa2_));
  gravity.set_enabled(iiwa2_, false);
  EXPECT_FALSE(gravity.is_enabled(iiwa1_));
  EXPECT_FALSE(gravity.is_enabled(iiwa2_));

  // Enable gravity.
  gravity.set_enabled(iiwa1_, true);
  EXPECT_TRUE(gravity.is_enabled(iiwa1_));
  EXPECT_FALSE(gravity.is_enabled(iiwa2_));
  gravity.set_enabled(iiwa2_, true);
  EXPECT_TRUE(gravity.is_enabled(iiwa1_));
  EXPECT_TRUE(gravity.is_enabled(iiwa2_));
}

TEST_F(MultibodyPlantGravityForceTest, InvalidModelInstancesThrow) {
  UniformGravityFieldElement<double>& gravity = plant_->mutable_gravity_field();
  const ModelInstanceIndex invalid_model_instance(
      plant_->num_model_instances());
  DRAKE_EXPECT_THROWS_MESSAGE(gravity.is_enabled(invalid_model_instance),
                              "Model instance index is invalid.");
  DRAKE_EXPECT_THROWS_MESSAGE(gravity.set_enabled(invalid_model_instance, true),
                              "Model instance index is invalid.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->is_gravity_enabled(invalid_model_instance),
      "Model instance index is invalid.");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->set_gravity_enabled(invalid_model_instance, true),
      "Model instance index is invalid.");
}

TEST_F(MultibodyPlantGravityForceTest,
       UniformGravityFieldElementSetEnabledThrowsIfCalledPostFinalize) {
  UniformGravityFieldElement<double>& gravity = plant_->mutable_gravity_field();
  plant_->Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(gravity.set_enabled(iiwa1_, true),
                              "Gravity can only be enabled pre-finalize.");
}

TEST_F(MultibodyPlantGravityForceTest,
       MultibodyPlantSetEnabledThrowsIfCalledPostFinalize) {
  // Pre-finalize we can call enable/disable as we please.
  EXPECT_NO_THROW(plant_->set_gravity_enabled(iiwa1_, true));

  // Verify that enabling/disabling gravity post-finalize throws.
  plant_->Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->set_gravity_enabled(iiwa1_, true),
      "Post-finalize calls to 'set_gravity_enabled\\(\\)' are not allowed; .*");
}

TEST_F(MultibodyPlantGravityForceTest, DisableGravity) {
  EXPECT_TRUE(plant_->is_gravity_enabled(iiwa1_));
  EXPECT_TRUE(plant_->is_gravity_enabled(iiwa2_));
  // We disable gravity on the first model only.
  plant_->set_gravity_enabled(iiwa1_, false);
  EXPECT_FALSE(plant_->is_gravity_enabled(iiwa1_));
  EXPECT_TRUE(plant_->is_gravity_enabled(iiwa2_));
  plant_->Finalize();
  std::unique_ptr<Context<double>> context = plant_->CreateDefaultContext();
  const VectorXd tau_g = plant_->CalcGravityGeneralizedForces(*context);
  const VectorXd tau_g1 = plant_->GetVelocitiesFromArray(iiwa1_, tau_g);
  EXPECT_EQ(tau_g1.norm(), 0.0);
  const VectorXd tau_g2 = plant_->GetVelocitiesFromArray(iiwa2_, tau_g);
  EXPECT_GT(tau_g2.norm(), 0.01 /* arbitrary, clearly non-zero, value */);
}

// Verify CalcGravityGeneralizedForces() against an analytical result for a
// non-planar (3D) double pendulum.
//   Link 1: mass m₁, COM at (0,0,−L₁) in body 1 frame, revolute about Y.
//   Link 2: mass m₂, COM at (0,0,−L₂) in body 2 frame, revolute about X,
//           joint located at (0,0,−d₁) in body 1 frame.
//
// Analysis by Claude (confirmed with MotionGenesis) shows the gravitational
// potential energy V is:
//   V = −m₁ g L₁ cos(q₁) − m₂ g (d₁ + L₂ cos(q₂)) cos(q₁)
//
// The generated generalized forces (tau_gᵢ = −∂V/∂qᵢ) are thus:
//   tau_g₁ = −g sin(q₁) (m₁ L₁ + m₂ (d₁ + L₂ cos(q₂)))
//   tau_g₂ = −m₂ g L₂ sin(q₂) cos(q₁)
GTEST_TEST(GravityTest, PotentialEnergyPowerAndGravityGeneralizedForces) {
  // Build a double pendulum from scratch.
  const double m1 = 2.0;  // kg
  const double m2 = 1.5;  // kg
  const double L1 = 0.5;  // m, joint 1 to COM of body 1
  const double d1 = 1.0;  // m, joint 1 to joint 2 along body 1
  const double L2 = 0.4;  // m, joint 2 to COM of body 2
  const double g = 9.81;  // m/s^2

  MultibodyPlant<double> pendulum(0.0);
  pendulum.mutable_gravity_field().set_gravity_vector(Vector3d(0, 0, -g));

  // Body 1 with COM at (0, 0, -L1) in body 1 frame.
  const SpatialInertia<double> M_B1 =
      SpatialInertia<double>::MakeFromCentralInertia(
          m1, Vector3d(0, 0, -L1), m1 * UnitInertia<double>::SolidSphere(0.1));
  const auto& body1 = pendulum.AddRigidBody("body1", M_B1);

  // Joint 1: revolute about Y connecting world to body 1.
  const auto& joint1 = pendulum.AddJoint<RevoluteJoint>(
      "joint1", pendulum.world_body(), {}, body1, {}, Vector3d::UnitY());

  // Body 2 with COM at (0, 0, -L2) in body 2 frame.
  const SpatialInertia<double> M_B2 =
      SpatialInertia<double>::MakeFromCentralInertia(
          m2, Vector3d(0, 0, -L2), m2 * UnitInertia<double>::SolidSphere(0.1));
  const auto& body2 = pendulum.AddRigidBody("body2", M_B2);

  // Joint 2: revolute about X, located at (0, 0, -d1) in body 1's frame.
  const auto& joint2 = pendulum.AddJoint<RevoluteJoint>(
      "joint2", body1, math::RigidTransformd(Vector3d(0, 0, -d1)), body2, {},
      Vector3d::UnitX());

  pendulum.Finalize();
  auto context = pendulum.CreateDefaultContext();

  // Test at several combinations of joint angles.
  for (const double q1 : {0.0, M_PI / 6, M_PI / 4, -M_PI / 3}) {
    for (const double q2 : {0.0, M_PI / 4, M_PI / 2, -M_PI / 6}) {
      SCOPED_TRACE(fmt::format("q1={}, q2={}", q1, q2));
      joint1.set_angle(context.get(), q1);
      joint2.set_angle(context.get(), q2);

      // Potential energy (see above).
      const double expected_V =
          -m1 * g * L1 * std::cos(q1) -
          m2 * g * (d1 + L2 * std::cos(q2)) * std::cos(q1);
      EXPECT_NEAR(pendulum.CalcPotentialEnergy(*context), expected_V, 1e-14);

      // Expected gravitational generalized forces (see above).
      const double expected_tau1 =
          -g * std::sin(q1) * (m1 * L1 + m2 * (d1 + L2 * std::cos(q2)));
      const double expected_tau2 = -m2 * g * L2 * std::sin(q2) * std::cos(q1);

      const VectorXd tau_g = pendulum.CalcGravityGeneralizedForces(*context);
      ASSERT_EQ(tau_g.size(), 2);
      EXPECT_NEAR(tau_g(0), expected_tau1, 1e-14);
      EXPECT_NEAR(tau_g(1), expected_tau2, 1e-14);
    }
  }

  // Conservative power P is related to potential energy V as P = -𝑑V/𝑑𝑡.
  // In this test, potential energy is only due to gravity and V = V(q₁,q₂), so
  // P = -𝑑V/𝑑𝑡 = -(∂V/q₁) q̇₁ - (∂V/q₂) q̇₂ = tau_g · v since the generalized
  // forces tau_g = -[∂V/q₁ ∂V/q₂] and generalized velocities are v = [q̇₁ q̇₂]ᵀ.
  // P is positive (increases kinetic energy) when potential energy V decreases.
  for (const double q1 : {M_PI / 6, -M_PI / 3}) {
    for (const double q2 : {M_PI / 4, -M_PI / 6}) {
      SCOPED_TRACE(fmt::format("q1={}, q2={}", q1, q2));
      joint1.set_angle(context.get(), q1);
      joint2.set_angle(context.get(), q2);

      const double v1 = 1.2;
      const double v2 = -0.7;
      joint1.set_angular_rate(context.get(), v1);
      joint2.set_angular_rate(context.get(), v2);

      const VectorXd tau_g = pendulum.CalcGravityGeneralizedForces(*context);
      const double expected_power = tau_g(0) * v1 + tau_g(1) * v2;
      EXPECT_NEAR(pendulum.EvalConservativePower(*context), expected_power,
                  1e-14);
    }
  }
}

}  // namespace
}  // namespace multibody
}  // namespace drake
