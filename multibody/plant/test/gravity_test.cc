#include <memory>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"

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
    const std::string full_name = FindResourceOrThrow(
        "drake/manipulation/models/iiwa_description/sdf/"
        "iiwa14_no_collision.sdf");
    plant_ = std::make_unique<MultibodyPlant<double>>(0.0);
    // Parser parser(plant_.get());

    // Add the model twice.
    iiwa1_ = Parser(plant_.get(), "iiwa1").AddModels(full_name).at(0);
    iiwa2_ = Parser(plant_.get(), "iiwa2").AddModels(full_name).at(0);
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
  gravity.disable(iiwa1_);
  EXPECT_FALSE(gravity.is_enabled(iiwa1_));
  EXPECT_TRUE(gravity.is_enabled(iiwa2_));
  gravity.disable(iiwa2_);
  EXPECT_FALSE(gravity.is_enabled(iiwa1_));
  EXPECT_FALSE(gravity.is_enabled(iiwa2_));

  // Enable gravity.
  gravity.enable(iiwa1_);
  EXPECT_TRUE(gravity.is_enabled(iiwa1_));
  EXPECT_FALSE(gravity.is_enabled(iiwa2_));
  gravity.enable(iiwa2_);
  EXPECT_TRUE(gravity.is_enabled(iiwa1_));
  EXPECT_TRUE(gravity.is_enabled(iiwa2_));
}

TEST_F(MultibodyPlantGravityForceTest,
       UniformGravityFieldElementThrowsIfDisabledPreFinalize) {
  UniformGravityFieldElement<double>& gravity = plant_->mutable_gravity_field();
  plant_->Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(gravity.enable(iiwa1_),
                              "Gravity can only be enabled post-finalize.");
  DRAKE_EXPECT_THROWS_MESSAGE(gravity.disable(iiwa1_),
                              "Gravity can only be disabled post-finalize.");
}

TEST_F(MultibodyPlantGravityForceTest, CantDisablePostFinalize) {
  // Pre-finalize we can call enable/disable as we please.
  EXPECT_NO_THROW(plant_->enable_gravity(iiwa1_));
  EXPECT_NO_THROW(plant_->disable_gravity(iiwa1_));

  // Verify that enabling/disabling gravity post-finalize throws.
  plant_->Finalize();
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->enable_gravity(iiwa1_),
      "Post-finalize calls to 'enable_gravity\\(\\)' are not allowed; .*");
  DRAKE_EXPECT_THROWS_MESSAGE(
      plant_->disable_gravity(iiwa2_),
      "Post-finalize calls to 'disable_gravity\\(\\)' are not allowed; .*");
}

TEST_F(MultibodyPlantGravityForceTest, DisableGravity) {
  EXPECT_TRUE(plant_->gravity_is_enabled(iiwa1_));
  EXPECT_TRUE(plant_->gravity_is_enabled(iiwa2_));
  // We disable gravity on the first model only.
  plant_->disable_gravity(iiwa1_);
  EXPECT_FALSE(plant_->gravity_is_enabled(iiwa1_));
  EXPECT_TRUE(plant_->gravity_is_enabled(iiwa2_));
  plant_->Finalize();
  std::unique_ptr<Context<double>> context = plant_->CreateDefaultContext();
  const VectorXd tau_g = plant_->CalcGravityGeneralizedForces(*context);
  const VectorXd tau_g1 = plant_->GetVelocitiesFromArray(iiwa1_, tau_g);
  EXPECT_EQ(tau_g1.norm(), 0.0);
  const VectorXd tau_g2 = plant_->GetVelocitiesFromArray(iiwa2_, tau_g);
  EXPECT_GT(tau_g2.norm(), 0.01 /* arbitrary, clearly non-zero, value */);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
